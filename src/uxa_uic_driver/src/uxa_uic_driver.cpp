#include "uxa_uic_driver.h"
#include "uic_packet.h"


int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("uxa_uic_driver");
    Init_Message(node);

    rclcpp::Rate loop_rate(1000);

    cout << "UIC_DRIVER : " <<  "UIC DRIVER stand by." << endl << endl;

    while(rclcpp::ok())
    {

        loop_rate.sleep();
        rclcpp::spin_some(node);
    }


    cout << endl;
    cout << "UIC_DRIVER : " << "uxa_uic_driver node terminate." << endl;
    return 0;
}
