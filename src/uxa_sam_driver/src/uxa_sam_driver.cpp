#include "uxa_sam_driver.h"

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("uxa_sam_driver");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    Init_Message(node);
    init_position_services();


    // rclcpp::Rate loop_rate(1000);

    cout << "SAM_DRIVER : " <<  "SAM DRIVER stand by.." << endl << endl;

    // while(rclcpp::ok())
    // {
    //     loop_rate.sleep();
    //     rclcpp::spin_some(node);
    // }

    executor.spin();
    rclcpp::shutdown();


    cout << endl;
    cout << "SAM_DRIVER : " << "uxa_sam_driver node terminate." << endl;
    return 0;
}
