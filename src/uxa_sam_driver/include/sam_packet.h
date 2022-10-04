#ifndef SAM_PACKET_H
#define SAM_PACKET_H

#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <uxa_serial_msgs/msg/receive.hpp>
#include <uxa_serial_msgs/msg/transmit.hpp>

#include <uxa_sam_msgs/msg/position_move.hpp>
#include <uxa_sam_msgs/msg/std_position_move.hpp>

using namespace std;

#define _MSG_BUFF_SIZE  20

void SERIAL_SUB_FUNC(const uxa_serial_msgs::msg::Receive::ConstSharedPtr &msg);
void SAM_POS_MOVE_FUNC(const uxa_sam_msgs::msg::PositionMove::ConstSharedPtr &msg);
void SAM_STD_POS_MOVE_FUNC(const uxa_sam_msgs::msg::StdPositionMove::ConstSharedPtr &msg);
void Init_Message(std::shared_ptr<rclcpp::Node>& n);
void Message_sender(unsigned char *Send_data, int Size);
void SAM_send_position(unsigned char id, unsigned char torq, unsigned char pos);
void SAM_send_std_position(unsigned char id, unsigned int pos);

#endif // SAM_PACKET_H
