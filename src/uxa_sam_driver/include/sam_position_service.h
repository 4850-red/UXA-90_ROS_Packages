#ifndef SAM_POSITION_SERVICE_H
#define SAM_POSITION_SERVICE_H

#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <uxa_sam_msgs/srv/position_move.hpp>
#include <uxa_serial_msgs/srv/serial.hpp>
#include <uxa_sam_msgs/srv/multi_move.hpp>

extern rclcpp::CallbackGroup::SharedPtr callback_group_position_service;
// extern rclcpp::Service<uxa_sam_msgs::srv::StdPositionMove>::SharedPtr uxa_std_position_service;
extern rclcpp::Service<uxa_sam_msgs::srv::PositionMove>::SharedPtr uxa_position_service;

void init_position_services();

void handlePositionMoveServiceRequest(
    const uxa_sam_msgs::srv::PositionMove::Request::SharedPtr request,
    uxa_sam_msgs::srv::PositionMove::Response::SharedPtr response
);

void handleMultiMoveServiceRequest(
    const uxa_sam_msgs::srv::MultiMove::Request::SharedPtr request,
    uxa_sam_msgs::srv::MultiMove::Response::SharedPtr response
);

#endif