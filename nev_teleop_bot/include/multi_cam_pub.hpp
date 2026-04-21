#pragma once

#include <atomic>
#include <cctype>
#include <chrono>
#include <fstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <nlohmann/json.hpp>

#include <cuda_runtime.h>
#include <nppi_geometry_transforms.h>

using ImageMsg = sensor_msgs::msg::Image;
using namespace std::chrono;

static constexpr int DEFAULT_CAM_WIDTH  = 1600;
static constexpr int DEFAULT_CAM_HEIGHT = 1200;
static constexpr int DEFAULT_CAM_FPS    = 15;

struct CamConfig {
    std::string device;
    std::string topic;
    std::string frame_id;
};

static const std::vector<CamConfig> CAMERAS = {
    {"/dev/cam_front",      "/camera/front/image_raw",      "cam_front"},
    {"/dev/cam_rear_left",  "/camera/rear_left/image_raw",  "cam_rear_left"},
    {"/dev/cam_rear_right", "/camera/rear_right/image_raw", "cam_rear_right"},
};

#define CUDA_CHECK(stmt)                                                       \
    do {                                                                       \
        cudaError_t _err = (stmt);                                             \
        if (_err != cudaSuccess)                                               \
            throw std::runtime_error(std::string("CUDA: ") +                   \
                                     cudaGetErrorString(_err));                \
    } while (0)

#define NPP_CHECK(stmt)                                                        \
    do {                                                                       \
        NppStatus _s = (stmt);                                                 \
        if (_s != NPP_SUCCESS)                                                 \
            throw std::runtime_error("NPP error: " + std::to_string(_s));      \
    } while (0)

class MultiCamPub : public rclcpp::Node
{
public:
    explicit MultiCamPub(const rclcpp::NodeOptions &options)
        : Node("multi_cam_pub", options)
    {
        gst_init(nullptr, nullptr);
        cv::setNumThreads(1);

        cam_w_   = declare_parameter<int>("cam_width",  DEFAULT_CAM_WIDTH);
        cam_h_   = declare_parameter<int>("cam_height", DEFAULT_CAM_HEIGHT);
        cam_fps_ = declare_parameter<int>("cam_fps",    DEFAULT_CAM_FPS);

        std::string default_calib =
            ament_index_cpp::get_package_share_directory("nev_teleop_bot") +
            "/param/ELP-USB16MP01-BL180-2048x1536_calibration.json";
        std::string calib_path =
            declare_parameter<std::string>("calib_path", default_calib);

        RCLCPP_INFO(get_logger(), "Building fisheye undistort maps (capture: %dx%d@%dfps)...",
                     cam_w_, cam_h_, cam_fps_);
        build_undistort_maps(calib_path);
        setup_gpu_resources();
        RCLCPP_INFO(get_logger(), "GPU remap ready (NPP CUDA backend)");

        auto qos = rclcpp::QoS(10).reliable().durability_volatile();

        for (size_t i = 0; i < CAMERAS.size(); i++) {
            pubs_.push_back(create_publisher<ImageMsg>(CAMERAS[i].topic, qos));
            open_camera(i);
            threads_.emplace_back(&MultiCamPub::capture_loop, this, i);
        }
        RCLCPP_INFO(get_logger(), "All %zu cameras started", CAMERAS.size());
    }

    ~MultiCamPub() override
    {
        running_ = false;
        for (auto &t : threads_)
            if (t.joinable()) t.join();
        for (size_t i = 0; i < pipelines_.size(); i++) {
            gst_element_set_state(pipelines_[i], GST_STATE_NULL);
            gst_object_unref(sinks_[i]);
            gst_object_unref(pipelines_[i]);
        }
        teardown_gpu_resources();
    }

private:
    void build_undistort_maps(const std::string &path)
    {
        std::ifstream f(path);
        if (!f.is_open())
            throw std::runtime_error("Cannot open calibration: " + path);

        auto j = nlohmann::json::parse(f);
        auto cam_key = j.begin().key();
        auto &intr = j[cam_key]["Intrinsic"];

        int calib_w = 0, calib_h = 0;
        auto xpos = cam_key.rfind('x');
        if (xpos != std::string::npos) {
            size_t wstart = xpos;
            while (wstart > 0 && std::isdigit(static_cast<unsigned char>(cam_key[wstart - 1]))) --wstart;
            size_t hend = xpos + 1;
            while (hend < cam_key.size() && std::isdigit(static_cast<unsigned char>(cam_key[hend]))) ++hend;
            calib_w = std::stoi(cam_key.substr(wstart, xpos - wstart));
            calib_h = std::stoi(cam_key.substr(xpos + 1, hend - xpos - 1));
        }
        if (calib_w == 0 || calib_h == 0)
            throw std::runtime_error("Cannot parse calibration resolution from key: " + cam_key);

        RCLCPP_INFO(get_logger(), "Calibration: %dx%d -> Capture: %dx%d (scale %.4f)",
                     calib_w, calib_h, cam_w_, cam_h_,
                     static_cast<double>(cam_w_) / calib_w);

        auto k_vec = intr["K"].get<std::vector<double>>();
        cv::Mat K = cv::Mat(3, 3, CV_64F, k_vec.data()).clone();

        auto d_vec = intr["D"].get<std::vector<double>>();
        cv::Mat D(4, 1, CV_64F);
        for (int i = 0; i < 4; i++) D.at<double>(i) = d_vec[i + 1];

        double sx = static_cast<double>(cam_w_) / calib_w;
        double sy = static_cast<double>(cam_h_) / calib_h;
        cv::Mat K_scaled = K.clone();
        K_scaled.at<double>(0, 0) *= sx;
        K_scaled.at<double>(0, 2) *= sx;
        K_scaled.at<double>(1, 1) *= sy;
        K_scaled.at<double>(1, 2) *= sy;

        cv::fisheye::initUndistortRectifyMap(
            K_scaled, D, cv::Mat::eye(3, 3, CV_64F), K_scaled,
            cv::Size(cam_w_, cam_h_), CV_32FC1, map_x_host_, map_y_host_);
    }

    void setup_gpu_resources()
    {
        const size_t frame_bytes = static_cast<size_t>(cam_w_) * cam_h_ * 3;
        const size_t map_bytes   = static_cast<size_t>(cam_w_) * cam_h_ * sizeof(float);

        CUDA_CHECK(cudaMalloc(&d_map_x_, map_bytes));
        CUDA_CHECK(cudaMalloc(&d_map_y_, map_bytes));
        CUDA_CHECK(cudaMemcpy(d_map_x_, map_x_host_.ptr<float>(), map_bytes, cudaMemcpyHostToDevice));
        CUDA_CHECK(cudaMemcpy(d_map_y_, map_y_host_.ptr<float>(), map_bytes, cudaMemcpyHostToDevice));
        map_step_ = cam_w_ * static_cast<int>(sizeof(float));

        d_src_.resize(CAMERAS.size(), nullptr);
        d_dst_.resize(CAMERAS.size(), nullptr);
        streams_.resize(CAMERAS.size(), nullptr);
        npp_ctx_.resize(CAMERAS.size());
        for (size_t i = 0; i < CAMERAS.size(); i++) {
            CUDA_CHECK(cudaMalloc(&d_src_[i], frame_bytes));
            CUDA_CHECK(cudaMalloc(&d_dst_[i], frame_bytes));
            CUDA_CHECK(cudaStreamCreate(&streams_[i]));

            NppStreamContext ctx = {};
            int dev = 0;
            CUDA_CHECK(cudaGetDevice(&dev));
            cudaDeviceProp prop;
            CUDA_CHECK(cudaGetDeviceProperties(&prop, dev));
            ctx.hStream                  = streams_[i];
            ctx.nCudaDeviceId            = dev;
            ctx.nMultiProcessorCount     = prop.multiProcessorCount;
            ctx.nMaxThreadsPerMultiProcessor = prop.maxThreadsPerMultiProcessor;
            ctx.nMaxThreadsPerBlock      = prop.maxThreadsPerBlock;
            ctx.nSharedMemPerBlock       = prop.sharedMemPerBlock;
            ctx.nCudaDevAttrComputeCapabilityMajor = prop.major;
            ctx.nCudaDevAttrComputeCapabilityMinor = prop.minor;
            ctx.nStreamFlags             = 0;
            npp_ctx_[i] = ctx;
        }
    }

    void teardown_gpu_resources()
    {
        for (size_t i = 0; i < d_src_.size(); i++) {
            if (d_src_[i]) cudaFree(d_src_[i]);
            if (d_dst_[i]) cudaFree(d_dst_[i]);
            if (streams_[i]) cudaStreamDestroy(streams_[i]);
        }
        if (d_map_x_) cudaFree(d_map_x_);
        if (d_map_y_) cudaFree(d_map_y_);
    }

    void open_camera(size_t idx)
    {
        std::string ps =
            "v4l2src device=" + CAMERAS[idx].device + " ! "
            "image/jpeg,width=" + std::to_string(cam_w_) +
            ",height=" + std::to_string(cam_h_) +
            ",framerate=" + std::to_string(cam_fps_) + "/1 ! "
            "jpegdec ! videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink name=sink emit-signals=false drop=true max-buffers=1 sync=false";

        GError *err = nullptr;
        auto *pipeline = gst_parse_launch(ps.c_str(), &err);
        if (!pipeline || err) {
            std::string e = err ? err->message : "unknown";
            if (err) g_error_free(err);
            throw std::runtime_error("Pipeline failed for " + CAMERAS[idx].device + ": " + e);
        }

        auto *sink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
        gst_element_set_state(pipeline, GST_STATE_PLAYING);

        auto *bus = gst_element_get_bus(pipeline);
        auto *msg = gst_bus_timed_pop_filtered(bus, 5 * GST_SECOND,
            static_cast<GstMessageType>(GST_MESSAGE_STATE_CHANGED | GST_MESSAGE_ERROR));
        if (msg && GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
            GError *gerr = nullptr;
            gst_message_parse_error(msg, &gerr, nullptr);
            std::string e = gerr ? gerr->message : "unknown";
            if (gerr) g_error_free(gerr);
            gst_message_unref(msg);
            gst_object_unref(bus);
            throw std::runtime_error("Camera open failed: " + CAMERAS[idx].device + ": " + e);
        }
        if (msg) gst_message_unref(msg);
        gst_object_unref(bus);

        pipelines_.push_back(pipeline);
        sinks_.push_back(sink);

        RCLCPP_INFO(get_logger(), "[%s] Opened %s — %dx%d@%dfps",
            CAMERAS[idx].frame_id.c_str(), CAMERAS[idx].device.c_str(),
            cam_w_, cam_h_, cam_fps_);
    }

    void capture_loop(size_t idx)
    {
        int count = 0;
        auto log_time = steady_clock::now();
        const size_t frame_bytes = static_cast<size_t>(cam_w_) * cam_h_ * 3;
        const int src_step = cam_w_ * 3;
        const NppiSize full_size = {cam_w_, cam_h_};
        const NppiRect src_roi   = {0, 0, cam_w_, cam_h_};

        while (running_) {
            auto t0 = steady_clock::now();
            GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sinks_[idx]));
            if (!sample) continue;

            GstBuffer *buf = gst_sample_get_buffer(sample);
            GstMapInfo map;
            if (!gst_buffer_map(buf, &map, GST_MAP_READ)) {
                gst_sample_unref(sample);
                continue;
            }

            auto t1 = steady_clock::now();
            auto msg = std::make_unique<ImageMsg>();
            msg->header.stamp = this->now();
            msg->header.frame_id = CAMERAS[idx].frame_id;
            msg->height = cam_h_;
            msg->width  = cam_w_;
            msg->encoding = "bgr8";
            msg->step = src_step;
            msg->data.resize(frame_bytes);

            CUDA_CHECK(cudaMemcpyAsync(d_src_[idx], map.data, frame_bytes,
                                       cudaMemcpyHostToDevice, streams_[idx]));
            NPP_CHECK(nppiRemap_8u_C3R_Ctx(
                static_cast<const Npp8u *>(d_src_[idx]), full_size, src_step, src_roi,
                static_cast<const Npp32f *>(d_map_x_), map_step_,
                static_cast<const Npp32f *>(d_map_y_), map_step_,
                static_cast<Npp8u *>(d_dst_[idx]), src_step, full_size,
                NPPI_INTER_LINEAR, npp_ctx_[idx]));
            CUDA_CHECK(cudaMemcpyAsync(msg->data.data(), d_dst_[idx], frame_bytes,
                                       cudaMemcpyDeviceToHost, streams_[idx]));
            CUDA_CHECK(cudaStreamSynchronize(streams_[idx]));
            auto t2 = steady_clock::now();

            pubs_[idx]->publish(std::move(msg));
            auto t3 = steady_clock::now();

            gst_buffer_unmap(buf, &map);
            gst_sample_unref(sample);
            count++;

            double elapsed = duration<double>(steady_clock::now() - log_time).count();
            if (elapsed >= 5.0) {
                RCLCPP_INFO(get_logger(),
                    "[%s] pull=%.1fms gpu=%.1fms pub=%.1fms total=%.1fms fps=%.1f",
                    CAMERAS[idx].frame_id.c_str(),
                    ms(t0, t1), ms(t1, t2), ms(t2, t3), ms(t0, t3),
                    count / elapsed);
                count = 0;
                log_time = steady_clock::now();
            }
        }
    }

    static double ms(steady_clock::time_point a, steady_clock::time_point b) {
        return duration<double, std::milli>(b - a).count();
    }

    std::atomic<bool> running_{true};
    int cam_w_, cam_h_, cam_fps_;

    cv::Mat map_x_host_, map_y_host_;
    void *d_map_x_ = nullptr;
    void *d_map_y_ = nullptr;
    int   map_step_ = 0;

    std::vector<void *> d_src_;
    std::vector<void *> d_dst_;
    std::vector<cudaStream_t> streams_;
    std::vector<NppStreamContext> npp_ctx_;

    std::vector<rclcpp::Publisher<ImageMsg>::SharedPtr> pubs_;
    std::vector<GstElement *> pipelines_;
    std::vector<GstElement *> sinks_;
    std::vector<std::thread> threads_;
};
