#include "sam_position_service.h"
#include "sam_resources.h"

rclcpp::Service<uxa_sam_msgs::srv::PositionMove>::SharedPtr uxa_position_service = nullptr;
rclcpp::CallbackGroup::SharedPtr callback_group_position_service = nullptr;

rclcpp::Service<uxa_sam_msgs::srv::MultiMove>::SharedPtr uxa_multimove_service = nullptr;
rclcpp::CallbackGroup::SharedPtr callback_group_multimove_service = nullptr;

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

    callback_group_multimove_service = node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive
    );

    uxa_multimove_service = node->create_service<uxa_sam_msgs::srv::MultiMove>(
        "uxa_sam_driver/services/multimove",
        &handleMultiMoveServiceRequest,
        rmw_qos_profile_default,
        callback_group_multimove_service
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
    serial_request->expect = 1;


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

void handleMultiMoveServiceRequest(
    const uxa_sam_msgs::srv::MultiMove::Request::SharedPtr request,
    uxa_sam_msgs::srv::MultiMove::Response::SharedPtr response
) {
    if (request->positions.size() == 0)
    {
        RCLCPP_ERROR(node->get_logger(), "RECEIVED AN EMPTY POSITION MULTIMOVE REQUEST");
        response->currents[0] = 0;
        response->positions[0] = 0;
        return;
    }

    uint positionCount = request->positions.size();
    RCLCPP_INFO(node->get_logger(), "MULTIMOVE REQUEST:");
    for (uint i = 0; i < positionCount; i += 1) 
    {
        RCLCPP_INFO(node->get_logger(), 
            "\tRECEIVED SAM POSITION REQUEST: ID: %d, POS: %d, TORQ: %d", 
            request->positions[i].id, 
            request->positions[i].pos, 
            request->positions[i].torqlevel
        );
    }

    unsigned char *send_buf = new unsigned char[positionCount * 4];

    for (uint i = 0; i < positionCount; i += 1)
    {
        int shift = i * 4;
        unsigned char id = request->positions[i].id;
        unsigned char pos = request->positions[i].pos;
        unsigned char torq = request->positions[i].torqlevel;

        send_buf[shift] = 0xFF;
        send_buf[shift + 1] = id | (torq << 5);
        send_buf[shift + 2] = pos;
        send_buf[shift + 3] = (send_buf[1]^send_buf[2]) & 0x7F;
    }

    auto serial_request = std::make_shared<uxa_serial_msgs::srv::Serial::Request>();

    serial_request->tx_data = std::vector<unsigned char, std::allocator<unsigned char>>(send_buf, send_buf + positionCount * 4);
    serial_request->expect = 1;


    auto result = uxa_serial_serial_client->async_send_request(serial_request);

    auto two_seconds = std::chrono::system_clock::now() + std::chrono::seconds(2);
    if (result.wait_until(two_seconds) == std::future_status::ready) 
    {
        auto content = result.get();
        if (content->success)
        {
            if (content->rx_data.size() != positionCount * 2)
            {
                RCLCPP_ERROR(node->get_logger(), 
                    "MULTIMOVE: SERIAL SERVICE RETURNED SUCCESS BUT CONTENT != EXPECTED LENGTH, EXPECTED: %d, FOUND: %ld", 
                    positionCount * 2, 
                    content->rx_data.size()
                );

                for (uint i = 0; i < positionCount; i += 1)
                {
                    response->currents.push_back(0);
                    response->positions.push_back(0);
                }

                return;
            }

            RCLCPP_INFO(node->get_logger(), "SERIAL SERVICE RESPONSE: RESPONSE VALID, FORWARDING");

            for (uint i = 0; positionCount; i += 1)
            {
                response->currents.push_back(content->rx_data[i * 2]);
                response->positions.push_back(content->rx_data[i * 2 +1]);
            }
        } 
        else
        {
            RCLCPP_ERROR(node->get_logger(), "SERIAL SERVICE ERROR, SERVICE RETURNED SUCCESS: FALSE");
            
            for (uint i = 0; i < positionCount; i += 1)
            {
                response->currents.push_back(0);
                response->positions.push_back(0);
            }
        }
    } 
    else 
    {
        RCLCPP_ERROR(node->get_logger(), "SERIAL SERVICE ERROR, NO DATA DETECTED");
        
        for (uint i = 0; i < positionCount; i += 1)
        {
            response->currents.push_back(0);
            response->positions.push_back(0);
        }
    }

    delete []send_buf;
}