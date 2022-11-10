#ifndef SAM_RESOURCES_H
#define SAM_RESOURCES_H

#include <rclcpp/rclcpp.hpp>

#include <uxa_serial_msgs/msg/receive.hpp>
#include <uxa_serial_msgs/msg/transmit.hpp>
#include <uxa_serial_msgs/srv/status.hpp>
#include <uxa_serial_msgs/srv/serial.hpp>

#include <uxa_sam_msgs/msg/position_move.hpp>
#include <uxa_sam_msgs/msg/std_position_move.hpp>
#include <uxa_sam_msgs/srv/status.hpp>

#include <uxa_sam_msgs/srv/position_move.hpp>
#include <uxa_sam_msgs/srv/std_position_move.hpp>



extern rclcpp::Node::SharedPtr node;

extern rclcpp::Publisher<uxa_serial_msgs::msg::Transmit, std::allocator<void>>::SharedPtr uxa_serial_pub;
extern rclcpp::Subscription<uxa_serial_msgs::msg::Receive>::SharedPtr uxa_serial_sub;
extern rclcpp::Subscription<uxa_sam_msgs::msg::PositionMove>::SharedPtr sam_driver_position_move_sub;
extern rclcpp::Subscription<uxa_sam_msgs::msg::StdPositionMove>::SharedPtr  sam_driver_std_position_move_sub;
extern rclcpp::Client<uxa_serial_msgs::srv::Status>::SharedPtr uxa_serial_status_client;
extern rclcpp::Service<uxa_sam_msgs::srv::Status>::SharedPtr uxa_sam_driver_status_service;
extern rclcpp::CallbackGroup::SharedPtr callback_group_status_client;


extern rclcpp::Client<uxa_serial_msgs::srv::Serial>::SharedPtr uxa_serial_serial_client;


#endif