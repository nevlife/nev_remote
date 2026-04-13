#include "video_bridge_base.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoBridgeBase>(
        "video_bridge", "h265", "nvh265enc", "video/x-h265,stream-format=byte-stream,alignment=au"));
    rclcpp::shutdown();
    return 0;
}
