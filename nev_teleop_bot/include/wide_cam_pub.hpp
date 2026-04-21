#pragma once

#include <chrono>
#include <cstring>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using ImageMsg = sensor_msgs::msg::Image;
using namespace std::chrono;

class WideCamPub : public rclcpp::Node
{
public:
    explicit WideCamPub(const rclcpp::NodeOptions &options)
        : Node("wide_cam_pub", options)
    {
        std::string topic_left  = declare_parameter<std::string>(
            "topic_left",  "/camera/rear_left/image_raw");
        std::string topic_front = declare_parameter<std::string>(
            "topic_front", "/camera/front/image_raw");
        std::string topic_right = declare_parameter<std::string>(
            "topic_right", "/camera/rear_right/image_raw");
        std::string topic_out   = declare_parameter<std::string>(
            "topic_out",   "/camera/wide/image_raw");
        int sync_queue = declare_parameter<int>("sync_queue", 20);
        bool reliable  = declare_parameter<bool>("reliable", true);

        sub_left_.subscribe (this, topic_left,  rmw_qos_profile_default);
        sub_front_.subscribe(this, topic_front, rmw_qos_profile_default);
        sub_right_.subscribe(this, topic_right, rmw_qos_profile_default);

        sync_ = std::make_shared<Sync>(SyncPolicy(sync_queue),
                                       sub_left_, sub_front_, sub_right_);
        sync_->registerCallback(std::bind(&WideCamPub::on_synced, this,
                                          std::placeholders::_1,
                                          std::placeholders::_2,
                                          std::placeholders::_3));

        auto qos = rclcpp::QoS(5).durability_volatile();
        if (reliable) qos.reliable();
        else          qos.best_effort();
        pub_ = create_publisher<ImageMsg>(topic_out, qos);

        RCLCPP_INFO(get_logger(),
            "WideCamPub: [%s | %s | %s] -> %s (QoS=%s)",
            topic_left.c_str(), topic_front.c_str(),
            topic_right.c_str(), topic_out.c_str(),
            reliable ? "reliable" : "best_effort");
    }

private:
    void on_synced(const ImageMsg::ConstSharedPtr &left,
                   const ImageMsg::ConstSharedPtr &front,
                   const ImageMsg::ConstSharedPtr &right)
    {
        auto t0 = steady_clock::now();

        if (left->encoding != "bgr8" || front->encoding != "bgr8" ||
            right->encoding != "bgr8") {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Unsupported encoding (need bgr8)");
            return;
        }
        if (left->width  != front->width  || front->width  != right->width ||
            left->height != front->height || front->height != right->height) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Image size mismatch");
            return;
        }

        const uint32_t W = left->width;
        const uint32_t H = left->height;
        const size_t   row_bytes = static_cast<size_t>(W) * 3;
        const uint32_t wide_w    = W * 3;
        const size_t   wide_step = static_cast<size_t>(wide_w) * 3;

        auto msg = std::make_unique<ImageMsg>();
        msg->header.stamp    = front->header.stamp;
        msg->header.frame_id = "cam_wide";
        msg->height   = H;
        msg->width    = wide_w;
        msg->encoding = "bgr8";
        msg->step     = wide_step;
        msg->data.resize(wide_step * H);

        uint8_t *dst = msg->data.data();
        const uint8_t *sl = left->data.data();
        const uint8_t *sf = front->data.data();
        const uint8_t *sr = right->data.data();

        for (uint32_t y = 0; y < H; y++) {
            std::memcpy(dst,                     sl + y * left->step,  row_bytes);
            std::memcpy(dst +     row_bytes,     sf + y * front->step, row_bytes);
            std::memcpy(dst + 2 * row_bytes,     sr + y * right->step, row_bytes);
            dst += wide_step;
        }

        pub_->publish(std::move(msg));

        count_++;
        double elapsed = duration<double>(steady_clock::now() - log_time_).count();
        if (elapsed >= 5.0) {
            auto t1 = steady_clock::now();
            RCLCPP_INFO(get_logger(),
                "wide %ux%u concat=%.1fms fps=%.1f",
                wide_w, H,
                duration<double, std::milli>(t1 - t0).count(),
                count_ / elapsed);
            count_ = 0;
            log_time_ = steady_clock::now();
        }
    }

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        ImageMsg, ImageMsg, ImageMsg>;
    using Sync = message_filters::Synchronizer<SyncPolicy>;

    message_filters::Subscriber<ImageMsg> sub_left_, sub_front_, sub_right_;
    std::shared_ptr<Sync> sync_;
    rclcpp::Publisher<ImageMsg>::SharedPtr pub_;

    int count_ = 0;
    steady_clock::time_point log_time_ = steady_clock::now();
};
