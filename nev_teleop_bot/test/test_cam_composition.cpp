#include "multi_cam_pub.hpp"
#include "stitch_cam_pub.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions opts;
    opts.use_intra_process_comms(true);

    auto multi_cam = std::make_shared<MultiCamPub>(opts);
    auto stitch    = std::make_shared<StitchCamPub>(opts);

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(multi_cam);
    exec.add_node(stitch);

    RCLCPP_INFO(multi_cam->get_logger(), "Composition started (intra-process ON)");
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
