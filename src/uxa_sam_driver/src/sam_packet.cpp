#include "sam_packet.h"
#include "sam_resources.h"


uxa_serial_msgs::msg::Transmit serial_pub_msg;

rclcpp::Node::SharedPtr node = nullptr;

rclcpp::Publisher<uxa_serial_msgs::msg::Transmit, std::allocator<void>>::SharedPtr uxa_serial_pub = nullptr;
rclcpp::Subscription<uxa_serial_msgs::msg::Receive>::SharedPtr uxa_serial_sub = nullptr;
rclcpp::Subscription<uxa_sam_msgs::msg::PositionMove>::SharedPtr sam_driver_position_move_sub = nullptr;
rclcpp::Subscription<uxa_sam_msgs::msg::StdPositionMove>::SharedPtr  sam_driver_std_position_move_sub = nullptr;


rclcpp::Service<uxa_sam_msgs::srv::Status>::SharedPtr uxa_sam_driver_status_service = nullptr;
rclcpp::CallbackGroup::SharedPtr callback_group_status_client = nullptr;
rclcpp::Client<uxa_serial_msgs::srv::Status>::SharedPtr uxa_serial_status_client = nullptr;

rclcpp::Client<uxa_serial_msgs::srv::Serial>::SharedPtr uxa_serial_serial_client = nullptr;
rclcpp::CallbackGroup::SharedPtr callback_group_serial_client = nullptr;

unsigned char Send_buf[_MSG_BUFF_SIZE];

using namespace std;

void SERIAL_SUB_FUNC(const uxa_serial_msgs::msg::Receive::ConstSharedPtr &msg)
{
    RCLCPP_INFO(node->get_logger(), "recieve msg : 0x%x",msg->rx_data);
}

void SAM_POS_MOVE_FUNC(const uxa_sam_msgs::msg::PositionMove::ConstSharedPtr &msg)
{
    RCLCPP_INFO(node->get_logger(), "recieve msg: ID: %u, Pos: %u, Torq: %u",msg->id, msg->pos, msg->torqlevel);
    SAM_send_position(msg->id, msg->torqlevel, msg->pos);
}

void SAM_STD_POS_MOVE_FUNC(const uxa_sam_msgs::msg::StdPositionMove::ConstSharedPtr &msg)
{
    RCLCPP_INFO(node->get_logger(), "recieve msg: ID: %u, Pos: %u", msg->id, msg->pos14);
    SAM_send_std_position(msg->id, msg->pos14);
}

void Init_Message(std::shared_ptr<rclcpp::Node>& n)
{
    node = n;

    uxa_serial_pub = node->create_publisher<uxa_serial_msgs::msg::Transmit>("uxa_serial_subscriber", _MSG_BUFF_SIZE);

    uxa_serial_sub = node->create_subscription<uxa_serial_msgs::msg::Receive>("uxa_serial_publisher", _MSG_BUFF_SIZE, SERIAL_SUB_FUNC);

    sam_driver_position_move_sub = node->create_subscription<uxa_sam_msgs::msg::PositionMove>
                ("uxa_sam_driver/position_move", _MSG_BUFF_SIZE, SAM_POS_MOVE_FUNC);

    sam_driver_std_position_move_sub = node->create_subscription<uxa_sam_msgs::msg::StdPositionMove>
                ("sam_driver_std_position_move", _MSG_BUFF_SIZE, SAM_STD_POS_MOVE_FUNC);

    uxa_sam_driver_status_service = node->create_service<uxa_sam_msgs::srv::Status>
                ("uxa_sam_driver/status", &handleStatusServiceRequest);
    RCLCPP_INFO(node->get_logger(), "Node ID %s", uxa_sam_driver_status_service->get_service_name());

    callback_group_status_client = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    uxa_serial_status_client = node->create_client<uxa_serial_msgs::srv::Status>(
        "uxa_serial/status", 
        rmw_qos_profile_default, 
        callback_group_status_client
    );

    init_serial_client();
}

void init_serial_client() 
{
    callback_group_serial_client = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    uxa_serial_serial_client = node->create_client<uxa_serial_msgs::srv::Serial>(
        "uxa_serial/serial",
        rmw_qos_profile_default,
        callback_group_serial_client
    );
}

void Message_sender(unsigned char *Send_data, int Size)
{
    serial_pub_msg.tx_data = std::vector<unsigned char, std::allocator<unsigned char>>(Send_data, Send_data + Size);
    uxa_serial_pub->publish(serial_pub_msg);
}

void SAM_send_position(unsigned char id, unsigned char torq, unsigned char pos)
{
    unsigned char cnt = 0;

    Send_buf[cnt++] = 0xFF;
    Send_buf[cnt++] = id | (torq << 5);
    Send_buf[cnt++] = pos;
    Send_buf[cnt++] = (Send_buf[1]^Send_buf[2]) & 0x7F;

    Message_sender(Send_buf, cnt);
}

void SAM_send_std_position(unsigned char id, unsigned int pos14)
{
    unsigned char cnt = 0;

    Send_buf[cnt++] = 0xFF;
    Send_buf[cnt++] = (unsigned char)(7 << 5);
    Send_buf[cnt++] = 200;
    Send_buf[cnt++] = id;
    Send_buf[cnt++] = pos14 >> 7;
    Send_buf[cnt++] = (unsigned char)(pos14 & 0x7F);
    Send_buf[cnt++] = (Send_buf[1]^Send_buf[2]^Send_buf[3]^Send_buf[4]^Send_buf[5]) & 0x7F;

    Message_sender(Send_buf, cnt);
}

void handleStatusServiceRequest(
    const std::shared_ptr<uxa_sam_msgs::srv::Status::Request> request, 
    std::shared_ptr<uxa_sam_msgs::srv::Status::Response> response
) {
    auto serial_request = std::make_shared<uxa_serial_msgs::srv::Status::Request>();
    if (!uxa_serial_status_client->wait_for_service(1s)) {
        RCLCPP_ERROR(node->get_logger(), "SERIAL STATUS SERVICE UNAVAILABLE, SAM_DRIVER STATUS: 0");
        response->status = false;
        return;
    }

    auto result = uxa_serial_status_client->async_send_request(serial_request);
    
    auto two_seconds = std::chrono::system_clock::now() + std::chrono::seconds(2);
    if (result.wait_until(two_seconds) == std::future_status::ready) 
    {
        bool status = result.get()->status;
        RCLCPP_INFO(node->get_logger(), "SERIAL STATIC SERVICE RESPONSE: %d, FORWARDING", status);
        response->status = status;
    } 
    else 
    {
        RCLCPP_ERROR(node->get_logger(), "SERIAL STATUS SERVICE UNAVAILABLE, SAM_DRIVER STATUS: 0");
        response->status = false;
    }
}