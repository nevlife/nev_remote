#pragma once

#include <atomic>
#include <chrono>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <nlohmann/json.hpp>

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
    {"/dev/cam_front_left",  "/camera/front_left/image_raw",  "cam_front_left"},
    {"/dev/cam_side_left",   "/camera/side_left/image_raw",   "cam_side_left"},
    {"/dev/cam_rear",        "/camera/rear/image_raw",        "cam_rear"},
    {"/dev/cam_front_right", "/camera/front_right/image_raw", "cam_front_right"},
    {"/dev/cam_side_right",  "/camera/side_right/image_raw",  "cam_side_right"},
};

class MultiCamPub : public rclcpp::Node
{
public:
    explicit MultiCamPub(const rclcpp::NodeOptions &options)
        : Node("multi_cam_pub", options)
    {
        gst_init(nullptr, nullptr);

        cam_w_   = declare_parameter<int>("cam_width",  DEFAULT_CAM_WIDTH);
        cam_h_   = declare_parameter<int>("cam_height", DEFAULT_CAM_HEIGHT);
        cam_fps_ = declare_parameter<int>("cam_fps",    DEFAULT_CAM_FPS);

        std::string calib_path = std::string(getenv("HOME")) +
            "/SCV/src/teleop/nev_teleop_bot/nev_teleop_bot/param/"
            "ELP-USB16MP01-BL180-2048x1536_calibration.json";

        RCLCPP_INFO(get_logger(), "Building fisheye undistort maps (capture: %dx%d@%dfps)...",
                     cam_w_, cam_h_, cam_fps_);
        build_undistort_maps(calib_path);
        RCLCPP_INFO(get_logger(), "Undistort maps ready");

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

        // Parse calibration resolution from key (e.g. "...-2048x1536")
        int calib_w = 0, calib_h = 0;
        auto xpos = cam_key.rfind('x');
        if (xpos != std::string::npos) {
            size_t wstart = xpos;
            while (wstart > 0 && std::isdigit(cam_key[wstart - 1])) --wstart;
            size_t hend = xpos + 1;
            while (hend < cam_key.size() && std::isdigit(cam_key[hend])) ++hend;
            calib_w = std::stoi(cam_key.substr(wstart, xpos - wstart));
            calib_h = std::stoi(cam_key.substr(xpos + 1, hend - xpos - 1));
        }
        if (calib_w == 0 || calib_h == 0)
            throw std::runtime_error("Cannot parse calibration resolution from key: " + cam_key);

        RCLCPP_INFO(get_logger(), "Calibration: %dx%d -> Capture: %dx%d (scale %.4f)",
                     calib_w, calib_h, cam_w_, cam_h_,
                     static_cast<double>(cam_w_) / calib_w);

        auto k_vec = intr["K"].get<std::vector<double>>();
        cv::Mat K(3, 3, CV_64F, k_vec.data());
        K = K.clone();

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
            cv::Size(cam_w_, cam_h_), CV_16SC2, map1_, map2_);
    }

    void open_camera(size_t idx)
    {
        std::string ps =
            "v4l2src device=" + CAMERAS[idx].device + " ! "
            "image/jpeg,width=" + std::to_string(cam_w_) +
            ",height=" + std::to_string(cam_h_) +
            ",framerate=" + std::to_string(cam_fps_) + "/1 ! "
            "jpegdec ! videoflip method=rotate-180 ! videoconvert ! "
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
        const size_t frame_bytes = cam_w_ * cam_h_ * 3;

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

            cv::Mat frame(cam_h_, cam_w_, CV_8UC3, map.data);

            auto t1 = steady_clock::now();
            auto msg = std::make_unique<ImageMsg>();
            msg->header.stamp = this->now();
            msg->header.frame_id = CAMERAS[idx].frame_id;
            msg->height = cam_h_;
            msg->width  = cam_w_;
            msg->encoding = "bgr8";
            msg->step = cam_w_ * 3;
            msg->data.resize(frame_bytes);

            cv::Mat out(cam_h_, cam_w_, CV_8UC3, msg->data.data());
            cv::remap(frame, out, map1_, map2_, cv::INTER_LINEAR);
            auto t2 = steady_clock::now();

            pubs_[idx]->publish(std::move(msg));
            auto t3 = steady_clock::now();

            gst_buffer_unmap(buf, &map);
            gst_sample_unref(sample);
            count++;

            double elapsed = duration<double>(steady_clock::now() - log_time).count();
            if (elapsed >= 5.0) {
                RCLCPP_INFO(get_logger(),
                    "[%s] pull=%.1fms remap=%.1fms pub=%.1fms total=%.1fms fps=%.1f",
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
    cv::Mat map1_, map2_;
    std::vector<rclcpp::Publisher<ImageMsg>::SharedPtr> pubs_;
    std::vector<GstElement *> pipelines_;
    std::vector<GstElement *> sinks_;
    std::vector<std::thread> threads_;
};
