#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

using ImageMsg = sensor_msgs::msg::Image;
using namespace std::chrono;

static constexpr int STITCH_WIDTH  = 1600;
static constexpr int STITCH_HEIGHT = 1200;
static constexpr int STITCH_FPS    = 15;
static constexpr int NUM_CAMS      = 5;

static const std::vector<std::string> SRC_TOPICS = {
    "/camera/front_left/image_raw",
    "/camera/side_left/image_raw",
    "/camera/rear/image_raw",
    "/camera/front_right/image_raw",
    "/camera/side_right/image_raw",
};

static const std::string DST_TOPIC = "/camera/camera/color/image_raw";

class StitchCamPub : public rclcpp::Node
{
public:
    explicit StitchCamPub(const rclcpp::NodeOptions &options)
        : Node("stitch_cam_pub", options)
    {
        frames_.resize(NUM_CAMS);
        valid_.resize(NUM_CAMS, false);

        auto qos = rclcpp::QoS(10).reliable().durability_volatile();
        pub_ = create_publisher<ImageMsg>(DST_TOPIC, qos);

        for (int i = 0; i < NUM_CAMS; i++) {
            subs_.push_back(create_subscription<ImageMsg>(
                SRC_TOPICS[i], qos,
                [this, i](ImageMsg::UniquePtr msg) { on_image(i, std::move(msg)); }));
            RCLCPP_INFO(get_logger(), "Subscribed: %s", SRC_TOPICS[i].c_str());
        }

        pub_thread_ = std::thread(&StitchCamPub::publish_loop, this);
        RCLCPP_INFO(get_logger(), "Publishing stitched image to %s", DST_TOPIC.c_str());
    }

    ~StitchCamPub() override
    {
        running_ = false;
        if (pub_thread_.joinable()) pub_thread_.join();
    }

private:
    void on_image(int idx, ImageMsg::UniquePtr msg)
    {
        cv::Mat frame(msg->height, msg->width, CV_8UC3, msg->data.data());

        std::lock_guard<std::mutex> lk(mtx_);
        frames_[idx] = frame.clone();
        valid_[idx] = true;
    }

    void publish_loop()
    {
        int count = 0;
        auto log_time = steady_clock::now();
        const int stitch_w = STITCH_WIDTH * NUM_CAMS;
        const size_t frame_bytes = stitch_w * STITCH_HEIGHT * 3;

        while (running_) {
            auto t0 = steady_clock::now();

            std::vector<cv::Mat> local;
            {
                std::lock_guard<std::mutex> lk(mtx_);
                bool all = true;
                for (int i = 0; i < NUM_CAMS; i++) {
                    if (!valid_[i]) { all = false; break; }
                }
                if (!all) {
                    std::this_thread::sleep_for(milliseconds(10));
                    continue;
                }
                local = frames_;
            }

            auto t1 = steady_clock::now();
            cv::Mat stitched;
            cv::hconcat(local, stitched);
            auto t2 = steady_clock::now();

            auto msg = std::make_unique<ImageMsg>();
            msg->header.stamp = this->now();
            msg->header.frame_id = "camera_stitched";
            msg->height = STITCH_HEIGHT;
            msg->width  = stitch_w;
            msg->encoding = "bgr8";
            msg->step = stitch_w * 3;
            msg->data.assign(stitched.data, stitched.data + frame_bytes);
            auto t3 = steady_clock::now();

            pub_->publish(std::move(msg));
            auto t4 = steady_clock::now();

            count++;
            double elapsed = duration<double>(steady_clock::now() - log_time).count();
            if (elapsed >= 5.0) {
                RCLCPP_INFO(get_logger(),
                    "[stitch] grab=%.1fms hconcat=%.1fms msg=%.1fms pub=%.1fms "
                    "total=%.1fms fps=%.1f size=%dx%d",
                    ms(t0, t1), ms(t1, t2), ms(t2, t3), ms(t3, t4), ms(t0, t4),
                    count / elapsed, stitch_w, STITCH_HEIGHT);
                count = 0;
                log_time = steady_clock::now();
            }

            double loop_s = duration<double>(steady_clock::now() - t0).count();
            double sleep_s = (1.0 / STITCH_FPS) - loop_s;
            if (sleep_s > 0)
                std::this_thread::sleep_for(duration<double>(sleep_s));
        }
    }

    static double ms(steady_clock::time_point a, steady_clock::time_point b) {
        return duration<double, std::milli>(b - a).count();
    }

    std::atomic<bool> running_{true};
    std::mutex mtx_;
    std::vector<cv::Mat> frames_;
    std::vector<bool> valid_;
    rclcpp::Publisher<ImageMsg>::SharedPtr pub_;
    std::vector<rclcpp::Subscription<ImageMsg>::SharedPtr> subs_;
    std::thread pub_thread_;
};
