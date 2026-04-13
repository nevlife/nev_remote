#pragma once

#include <chrono>
#include <cstring>
#include <mutex>
#include <string>
#include <atomic>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include "zenoh.h"

using namespace std::chrono;
using ImageMsg = sensor_msgs::msg::Image;

inline double now_sec()  { return duration<double>(system_clock::now().time_since_epoch()).count(); }
inline double mono_sec() { return duration<double>(steady_clock::now().time_since_epoch()).count(); }

class VideoBridgeBase : public rclcpp::Node
{
public:
    VideoBridgeBase(const std::string &node_name, const std::string &codec,
                    const std::string &encoder_element, const std::string &caps_format)
        : Node(node_name), codec_(codec), encoder_element_(encoder_element), caps_format_(caps_format)
    {
        declare_parameter("image_topic", "/camera/camera/color/image_raw");
        declare_parameter("video_locator", "tcp/127.0.0.1:7447");
        declare_parameter("width", 1280);
        declare_parameter("height", 720);
        declare_parameter("max_fps", 30.0);
        declare_parameter("n-threads", 4);
        declare_parameter("hw_accel", true);
        declare_parameter("preset", "low-latency-hp");
        declare_parameter("rc-mode", "cbr");
        declare_parameter("bitrate", 1000);
        declare_parameter("max-bitrate", 0);
        declare_parameter("gop-size", 60);
        declare_parameter("aud", false);
        declare_parameter("gst_qos", false);
        declare_parameter("zerolatency", true);
        declare_parameter("rc-lookahead", 0);
        declare_parameter("b-frames", 0);
        declare_parameter("config-interval", -1);

        image_topic_    = get_parameter("image_topic").as_string();
        zenoh_locator_  = get_parameter("video_locator").as_string();
        target_w_       = get_parameter("width").as_int();
        target_h_       = get_parameter("height").as_int();
        max_fps_        = get_parameter("max_fps").as_double();
        frame_interval_ = 1.0 / max_fps_;

        gst_init(nullptr, nullptr);
        init_zenoh();

        auto qos = rclcpp::QoS(5).best_effort();
        sub_ = create_subscription<ImageMsg>(
            image_topic_, qos,
            [this](const ImageMsg::SharedPtr msg) { on_image(msg); });

        RCLCPP_INFO(get_logger(), "%s ready — topic: %s", node_name.c_str(), image_topic_.c_str());
    }

    ~VideoBridgeBase() override
    {
        if (pipeline_) { gst_element_set_state(pipeline_, GST_STATE_NULL); gst_object_unref(pipeline_); }
        z_drop(z_move(zpub_cam_));
        z_drop(z_move(zpub_stats_));
        z_drop(z_move(zsession_));
    }

private:
    void init_zenoh()
    {
        z_owned_config_t zconf;
        z_config_default(&zconf);
        if (!zenoh_locator_.empty()) {
            std::string ep = "[\"" + zenoh_locator_ + "\"]";
            zc_config_insert_json5(z_loan_mut(zconf), "connect/endpoints", ep.c_str());
        }
        if (z_open(&zsession_, z_move(zconf), NULL) < 0)
            throw std::runtime_error("Zenoh connect failed");
        RCLCPP_INFO(get_logger(), "Zenoh → %s", zenoh_locator_.empty() ? "auto" : zenoh_locator_.c_str());

        z_view_keyexpr_t ke_cam, ke_stats;
        z_view_keyexpr_from_str(&ke_cam, "nev/robot/camera");
        z_view_keyexpr_from_str(&ke_stats, "nev/robot/video_stats");

        z_publisher_options_t opts;
        z_publisher_options_default(&opts);
        opts.congestion_control = Z_CONGESTION_CONTROL_DROP;
        z_declare_publisher(z_loan(zsession_), &zpub_cam_, z_loan(ke_cam), &opts);
        z_declare_publisher(z_loan(zsession_), &zpub_stats_, z_loan(ke_stats), NULL);
    }

    void on_image(const ImageMsg::SharedPtr msg)
    {
        double now = mono_sec();
        if (now - last_push_ < frame_interval_) { drop_count_++; return; }
        double dt_ms = (now - last_push_) * 1000.0;
        last_push_ = now;
        cb_count_++;
        if (dt_ms > input_max_ms_) input_max_ms_ = dt_ms;

        if (now - log_time_ >= 2.0) {
            RCLCPP_INFO(get_logger(), "[enc] cb=%.1ffps drop=%d last_dt=%.0fms",
                        cb_count_ / (now - log_time_), drop_count_, dt_ms);
            cb_count_ = 0; drop_count_ = 0; log_time_ = now;
        }

        if (gst_fmt_.empty()) {
            gst_fmt_ = encoding_to_gst(msg->encoding);
            if (gst_fmt_.empty()) return;
        }
        if (!pipeline_) init_pipeline(msg->width, msg->height);

        auto msg_ref = new ImageMsg::SharedPtr(msg);
        GstBuffer *buf = gst_buffer_new_wrapped_full(
            static_cast<GstMemoryFlags>(0),
            const_cast<uint8_t*>(msg->data.data()), msg->data.size(), 0, msg->data.size(),
            msg_ref, [](gpointer p) { delete static_cast<ImageMsg::SharedPtr*>(p); });

        push_time_.store(mono_sec());
        gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buf);
    }

    void init_pipeline(int w, int h)
    {
        auto b = [this](const char *p) -> const char* { return get_parameter(p).as_bool() ? "true" : "false"; };
        auto s = [this](const char *p) { return get_parameter(p).as_string(); };
        auto i = [this](const char *p) { return get_parameter(p).as_int(); };
        int n_threads = (int)i("n-threads");
        bool hw_accel = get_parameter("hw_accel").as_bool();
        const char *parser = (codec_ == "h264") ? "h264parse" : "h265parse";
        long cfg_interval = i("config-interval");

        char ps[2048];
        if (hw_accel) {
            snprintf(ps, sizeof(ps),
                "appsrc name=appsrc format=time is-live=true do-timestamp=true "
                "caps=\"video/x-raw,format=%s,width=%d,height=%d,framerate=%d/1\" ! "
                "videoconvert n-threads=%d ! "
                "video/x-raw,format=NV12,width=%ld,height=%ld ! "
                "%s preset=%s rc-mode=%s bitrate=%ld max-bitrate=%ld "
                "gop-size=%ld aud=%s qos=%s zerolatency=%s "
                "rc-lookahead=%ld bframes=%ld ! "
                "%s config-interval=%ld ! "
                "%s ! appsink name=appsink drop=true max-buffers=1 sync=false",
                gst_fmt_.c_str(), w, h, (int)max_fps_,
                n_threads, target_w_, target_h_,
                encoder_element_.c_str(),
                s("preset").c_str(), s("rc-mode").c_str(),
                i("bitrate"), i("max-bitrate"),
                i("gop-size"), b("aud"), b("gst_qos"), b("zerolatency"),
                i("rc-lookahead"), i("b-frames"),
                parser, cfg_interval,
                caps_format_.c_str());
        } else if (codec_ == "h264") {
            snprintf(ps, sizeof(ps),
                "appsrc name=appsrc format=time is-live=true do-timestamp=true "
                "caps=\"video/x-raw,format=%s,width=%d,height=%d,framerate=%d/1\" ! "
                "videoconvert n-threads=%d ! "
                "video/x-raw,format=I420,width=%ld,height=%ld ! "
                "x264enc tune=zerolatency speed-preset=ultrafast bitrate=%ld "
                "key-int-max=%ld bframes=%ld byte-stream=true aud=%s ! "
                "h264parse config-interval=%ld ! "
                "%s ! appsink name=appsink drop=true max-buffers=1 sync=false",
                gst_fmt_.c_str(), w, h, (int)max_fps_,
                n_threads, target_w_, target_h_,
                i("bitrate"),
                i("gop-size"), i("b-frames"), b("aud"),
                cfg_interval,
                caps_format_.c_str());
        } else {
            snprintf(ps, sizeof(ps),
                "appsrc name=appsrc format=time is-live=true do-timestamp=true "
                "caps=\"video/x-raw,format=%s,width=%d,height=%d,framerate=%d/1\" ! "
                "videoconvert n-threads=%d ! "
                "video/x-raw,format=I420,width=%ld,height=%ld ! "
                "x265enc tune=zerolatency speed-preset=ultrafast bitrate=%ld "
                "key-int-max=%ld option-string=\"bframes=%ld\" ! "
                "h265parse config-interval=%ld ! "
                "%s ! appsink name=appsink drop=true max-buffers=1 sync=false",
                gst_fmt_.c_str(), w, h, (int)max_fps_,
                n_threads, target_w_, target_h_,
                i("bitrate"),
                i("gop-size"), i("b-frames"),
                cfg_interval,
                caps_format_.c_str());
        }

        RCLCPP_INFO(get_logger(), "pipeline: %s", ps);

        GError *err = nullptr;
        pipeline_ = gst_parse_launch(ps, &err);
        if (!pipeline_ || err) {
            if (err) { RCLCPP_FATAL(get_logger(), "Pipeline: %s", err->message); g_error_free(err); }
            throw std::runtime_error("Pipeline failed");
        }

        appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "appsrc");
        g_object_set(appsrc_, "max-bytes", (guint64)(w * h * 3 * 2), "block", FALSE, "leaky-type", 2, nullptr);

        auto *sink = gst_bin_get_by_name(GST_BIN(pipeline_), "appsink");
        GstAppSinkCallbacks cbs = {};
        cbs.new_sample = [](GstAppSink *s, gpointer ud) -> GstFlowReturn {
            return static_cast<VideoBridgeBase*>(ud)->on_encoded(s);
        };
        gst_app_sink_set_callbacks(GST_APP_SINK(sink), &cbs, this, nullptr);
        gst_object_unref(sink);

        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        RCLCPP_INFO(get_logger(), "Streaming: %ldx%ld @ %.0ffps", target_w_, target_h_, max_fps_);
    }

    GstFlowReturn on_encoded(GstAppSink *sink)
    {
        GstSample *sample = gst_app_sink_pull_sample(sink);
        if (!sample) return GST_FLOW_OK;

        double enc_ms = 0;
        double pt = push_time_.load();
        if (pt > 0) enc_ms = (mono_sec() - pt) * 1000.0;

        GstBuffer *buf = gst_sample_get_buffer(sample);
        GstMapInfo map;
        if (gst_buffer_map(buf, &map, GST_MAP_READ)) {
            double ts = now_sec();
            float em = static_cast<float>(enc_ms);
            size_t nal_sz = map.size, total = 12 + nal_sz;

            pkt_buf_.resize(total);
            std::memcpy(pkt_buf_.data(), &ts, 8);
            std::memcpy(pkt_buf_.data() + 8, &em, 4);
            std::memcpy(pkt_buf_.data() + 12, map.data, nal_sz);
            gst_buffer_unmap(buf, &map);

            auto t_put0 = steady_clock::now();
            z_owned_bytes_t payload;
            z_bytes_from_buf(&payload, pkt_buf_.data(), total, NULL, NULL);
            z_publisher_put(z_loan(zpub_cam_), z_move(payload), NULL);
            double put_ms = duration<double, std::milli>(steady_clock::now() - t_put0).count();

            tx_bytes_ += nal_sz; enc_sum_ += enc_ms; enc_count_++;
            nal_sum_ += nal_sz;
            if (enc_ms > enc_max_ms_) enc_max_ms_ = enc_ms;
            if (put_ms > put_max_ms_) put_max_ms_ = put_ms;
            if (nal_sz > nal_max_) nal_max_ = nal_sz;

            double now = now_sec();
            if (now - stats_ts_ >= 1.0) {
                double dt = now - stats_ts_;
                double bw = tx_bytes_ * 8.0 / (dt * 1e6);
                double enc_avg = enc_count_ > 0 ? enc_sum_ / enc_count_ : 0;
                double fps = enc_count_ / dt;
                size_t nal_avg = enc_count_ > 0 ? nal_sum_ / enc_count_ : 0;
                int drop = drop_count_;
                double enc_max = enc_max_ms_;
                double put_max = put_max_ms_;
                double in_max = input_max_ms_;
                size_t n_max = nal_max_;

                tx_bytes_ = 0; enc_sum_ = 0; enc_count_ = 0;
                nal_sum_ = 0; enc_max_ms_ = 0; put_max_ms_ = 0; nal_max_ = 0;
                input_max_ms_ = 0; drop_count_ = 0;
                stats_ts_ = now;

                char json[256];
                snprintf(json, sizeof(json),
                    "{\"codec\":\"%s\",\"bw_mbps\":%.3f,\"fps\":%.1f,\"drop\":%d,"
                    "\"enc_avg_ms\":%.2f,\"enc_max_ms\":%.1f,\"put_max_ms\":%.1f,"
                    "\"input_max_ms\":%.1f,"
                    "\"nal_avg\":%zu,\"nal_max\":%zu}",
                    codec_.c_str(), bw, fps, drop, enc_avg, enc_max, put_max, in_max, nal_avg, n_max);
                z_owned_bytes_t sp;
                z_bytes_copy_from_str(&sp, json);
                z_publisher_put(z_loan(zpub_stats_), z_move(sp), NULL);
            }
        }
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    std::string encoding_to_gst(const std::string &e)
    {
        if (e == "bgr8")  return "BGR";
        if (e == "rgb8")  return "RGB";
        if (e == "rgba8") return "RGBA";
        if (e == "bgra8") return "BGRA";
        if (e == "mono8") return "GRAY8";
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Unsupported: %s", e.c_str());
        return "";
    }

    std::string codec_, encoder_element_, caps_format_, gst_fmt_;
    std::string image_topic_, zenoh_locator_;
    int64_t target_w_, target_h_;
    double max_fps_, frame_interval_;
    double last_push_ = 0, log_time_ = 0;
    int cb_count_ = 0, drop_count_ = 0;
    std::atomic<double> push_time_{0};
    GstElement *pipeline_ = nullptr, *appsrc_ = nullptr;
    z_owned_session_t zsession_;
    z_owned_publisher_t zpub_cam_, zpub_stats_;
    std::vector<uint8_t> pkt_buf_;
    size_t tx_bytes_ = 0, nal_sum_ = 0, nal_max_ = 0;
    double enc_sum_ = 0, enc_max_ms_ = 0, put_max_ms_ = 0, input_max_ms_ = 0, stats_ts_ = 0;
    int enc_count_ = 0;
    rclcpp::Subscription<ImageMsg>::SharedPtr sub_;
};
