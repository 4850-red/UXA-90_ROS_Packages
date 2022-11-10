#include "sam_position_service.h"
#include "sam_resources.h"

rclcpp::Service<uxa_sam_msgs::srv::PositionMove>::SharedPtr uxa_position_service = nullptr;
rclcpp::CallbackGroup::SharedPtr callback_group_position_service = nullptr;

rclcpp::Publisher<uxa_sam_msgs::msg::PositionMove>::SharedPtr position_move_pub = nullptr;


void init_position_services()
{
    RCLCPP_INFO(node->get_logger(), "Initializing SAM Services...");

    position_move_pub = node->create_publisher<uxa_sam_msgs::msg::PositionMove>("uxa_sam_driver/position_move", 20);

    callback_group_position_service = node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive
    );
    
    uxa_position_service = node->create_service<uxa_sam_msgs::srv::PositionMove>(
        "uxa_sam_driver/services/position_move",
        &handlePositionMoveServiceRequest,
        rmw_qos_profile_default,
        callback_group_position_service
    );
}

void handlePositionMoveServiceRequest(
    const uxa_sam_msgs::srv::PositionMove::Request::SharedPtr request,
    uxa_sam_msgs::srv::PositionMove::Response::SharedPtr response
) {
    RCLCPP_INFO(node->get_logger(), "RECEIVED SAM POSITION REQUEST: ID: %d, POS: %d, TORQ: %d", request->id, request->pos, request->torqlevel);

    unsigned char *send_buf = new unsigned char[4];

    unsigned char id = request->id;
    unsigned char pos = request->pos;
    unsigned char torq = request->torqlevel;

    send_buf[0] = 0xFF;
    send_buf[1] = id | (torq << 5);
    send_buf[2] = pos;
    send_buf[3] = (send_buf[1]^send_buf[2]) & 0x7F;

    auto serial_request = std::make_shared<uxa_serial_msgs::srv::Serial::Request>();

    serial_request->tx_data = std::vector<unsigned char, std::allocator<unsigned char>>(send_buf, send_buf + 4);
    serial_request->expect = true;


    auto result = uxa_serial_serial_client->async_send_request(serial_request);

    auto two_seconds = std::chrono::system_clock::now() + std::chrono::seconds(2);
    if (result.wait_until(two_seconds) == std::future_status::ready) 
    {
        auto content = result.get();
        if (content->success)
        {
            RCLCPP_INFO(node->get_logger(), "SERIAL SERVICE RESPONSE: CURRENT: %d, POSITION: %d, TRANSMITTING", content->rx_data[0], content->rx_data[1]);
            response->current = content->rx_data[0];
            response->position = content->rx_data[1];
        } 
        else
        {
            RCLCPP_ERROR(node->get_logger(), "SERIAL SERVICE ERROR, SERVICE RETURNED SUCCESS: FALSE");
            response->current = 0;
            response->position = 0;
        }
    } 
    else 
    {
        RCLCPP_ERROR(node->get_logger(), "SERIAL SERVICE ERROR, NO DATA DETECTED");
        response->current = 0;
        response->position = 0;
    }

    delete []send_buf;
}