#include "video_bridge_base.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoBridgeBase>(
        "video_bridge_h264", "h264", "nvh264enc", "video/x-h264,stream-format=byte-stream,alignment=au"));
    rclcpp::shutdown();
    return 0;
}
