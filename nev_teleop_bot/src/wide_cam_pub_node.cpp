#include "wide_cam_pub.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WideCamPub>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
