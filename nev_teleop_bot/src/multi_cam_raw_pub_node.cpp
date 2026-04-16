#include "multi_cam_raw_pub.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiCamRawPub>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
