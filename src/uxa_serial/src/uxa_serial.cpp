#include "uxa_serial.h"

std::shared_ptr<rclcpp::Node> node;
volatile bool status = false;

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("uxa_serial");

    msg_buf = new unsigned char[_MSG_BUFF_SIZE];


    // ros::Publisher uxa_serial_pub = n.advertise<uxa_serial_msgs::receive>("uxa_serial_publisher", _MSG_BUFF_SIZE);
    // ros::Subscriber uxa_serial_sub = n.subscribe<uxa_serial_msgs::transmit>("uxa_serial_subscriber", _MSG_BUFF_SIZE, rev_func);

    auto uxa_serial_pub = node->create_publisher<uxa_serial_msgs::msg::Receive>("uxa_serial_publisher", _MSG_BUFF_SIZE);
    rclcpp::Rate loop_rate(1000);

    auto uxa_serial_sub = node->create_subscription<uxa_serial_msgs::msg::Transmit>("uxa_serial_subscriber", 10, rev_func);

    auto uxa_serial_status_service = node->create_service<uxa_serial_msgs::srv::Status>("uxa_serial/status", &handleStatusServiceRequest);

    // ros::Rate loop_rate();

    if((SerialMode = Init_Serial(_SERIAL_PORT)) != -1)
    {
        unsigned char Trans_chr[_SERIAL_BUFF_SIZE];
        unsigned char Recev_chr[_SERIAL_BUFF_SIZE];
        unsigned char cnt = 0;

        Trans_chr[cnt++] = 0xFF;
        Trans_chr[cnt++] = 0xFF;
        Trans_chr[cnt++] = 0xAA;
        Trans_chr[cnt++] = 0x55;
        Trans_chr[cnt++] = 0xAA;
        Trans_chr[cnt++] = 0x55;
        Trans_chr[cnt++] = 0x37;
        Trans_chr[cnt++] = 0xBA;
        Trans_chr[cnt++] = 0x10;
        Trans_chr[cnt++] = 0x00;
        Trans_chr[cnt++] = 0x00;
        Trans_chr[cnt++] = 0x00;
        Trans_chr[cnt++] = 0x00;
        Trans_chr[cnt++] = 0x01;
        Trans_chr[cnt++] = 0x01;
        Trans_chr[cnt++] = 0x01;

        Send_Serial_String(SerialMode, Trans_chr, cnt);
        rclcpp::spin_some(node);
        sleep(1);

        cnt = 0;

        Trans_chr[cnt++] = 0xFF;
        Trans_chr[cnt++] = (unsigned char)(7 << 5);
        Trans_chr[cnt++] = 225;
        Trans_chr[cnt++] = 0;
        Trans_chr[cnt++] = 0X07;
        Trans_chr[cnt++] = (Trans_chr[1]^Trans_chr[2]^Trans_chr[3]^Trans_chr[4]) & 0x7F;

        Send_Serial_String(SerialMode, Trans_chr, cnt);
        rclcpp::spin_some(node);
        sleep(1);

        memset(Trans_chr, '\0', sizeof(Trans_chr));
        memset(Recev_chr, '\0', sizeof(Recev_chr));

        cout << "SERIAL : " <<  "Serial communication stand by." << endl << endl;
        status = true;
        cout << "SERIAL STATUS: ENABLED" << endl;

        uxa_serial_msgs::msg::Receive msg;

        auto uxa_serial_service = node->create_service<uxa_serial_msgs::srv::Serial>("uxa_serial/serial", &handleSerialServiceRequest);

        while(rclcpp::ok())
        {
            loop_rate.sleep();
            if(Read_Serial_Char(SerialMode, Recev_chr) == 1)
            {
                msg.rx_data = Recev_chr[0];
                uxa_serial_pub->publish(msg);
                RCLCPP_WARN(node->get_logger(), "DANGLING SERIAL MSG FOUND!");
            }
            rclcpp::spin_some(node);
        }

    }

    cout << endl;
    cout << "SERIAL : " << SerialMode << " Device close." << endl;
    close(SerialMode);
    cout << "SERIAL : " << "uxa_serial node terminate." << endl;
    delete[] msg_buf;
    rclcpp::shutdown();
    return 0;
}


int Init_Serial(const char *Serial_Port)
{

    termios Serial_Setting;

    if((SerialMode = open(Serial_Port, O_RDWR | O_NONBLOCK | O_NOCTTY)) == -1)
    {
        cout << "SERIAL : " << Serial_Port << " Device open error" << endl;
        cout << "SERIAL : " << Serial_Port << " Device permission change progress...." << endl;
        for(int temp = 0; temp < 5; temp++)
        {
            if(chmod(Serial_Port, __S_IREAD | __S_IWRITE) == 0){

                cout << "SERIAL : " << Serial_Port << " Device permission change complete" << endl;
                SerialMode = open(Serial_Port, O_RDWR | O_NONBLOCK | O_NOCTTY);

                if(SerialMode == -1)
                {
                    cout << "SERIAL : " << Serial_Port << " Device Not Found" << endl;
                    return -1;
                }
                else
                    cout << "SERIAL : " << Serial_Port <<" Device open" << endl;
            }
            else
            {
                cout << "SERIAL : " << Serial_Port << " Device permission change error" << endl;
                //return -1;
            }
        }

    }

    else
        cout << "SERIAL : " << Serial_Port << " Device open" << endl;


    memset(&Serial_Setting, 0, sizeof(Serial_Setting));
    Serial_Setting.c_iflag = 0;
    Serial_Setting.c_oflag = 0;
    Serial_Setting.c_cflag = _BAUDRATE | CS8 | CREAD | CLOCAL;
    Serial_Setting.c_lflag = 0;
    Serial_Setting.c_cc[VMIN] = 1;
    Serial_Setting.c_cc[VTIME] = 0;

    cfsetispeed(&Serial_Setting, _BAUDRATE);
    cfsetospeed(&Serial_Setting, _BAUDRATE);

    tcflush(SerialMode, TCIOFLUSH); // originally TCIFLUSH
    tcsetattr(SerialMode, TCSANOW, &Serial_Setting);

    return SerialMode;

}

/*
int Serial_Test(int Serial, unsigned int Test_size)
{

    char Trans_chr[Test_size];
    char Recei_chr[Test_size];

    memset(Trans_chr, 0b10101010, sizeof(Trans_chr));
    memset(Recei_chr, '\0', sizeof(Recei_chr));

    write(Serial, Trans_chr, sizeof(Trans_chr));

    if(read(Serial, Recei_chr, sizeof(Recei_chr)) > 0)
    {
        cout << "Receive" << endl;
        if(strcmp(Trans_chr, Recei_chr) != 0)
        {
            close(Serial);
            cout << endl;
            cout << "ERR" << endl;
            cout << "ERR Receive CHR = " << Recei_chr << endl;
            return -1;
        }
        else
        {
            cout << endl;
            cout << "OK" <<endl;
            memset(Recei_chr, '\0', sizeof(Recei_chr));
        }

    }
}
*/

void Send_Serial_String(int Serial, unsigned char *Trans_chr, int Size)
{
    write(Serial, Trans_chr, Size);
}

void Send_Serial_Char(int Serial, unsigned char *Trans_chr)
{
    write(Serial, Trans_chr, 1);
}

int Read_Serial_Char(int Serial, unsigned char *Recei_chr)
{
    if(read(Serial, Recei_chr, 1) > 0)
        return 1;

    return -1;
}

int Read_Chars(int Serial, unsigned char *receive_chr, int count)
{
    int rec_count = read(Serial, receive_chr, count);
    RCLCPP_INFO(node->get_logger(), "Expected %d, Received %d characters", count, rec_count);
    if (count != rec_count)
        return -1;
    else
        return 1;
}

const std::string print_vector(const std::vector<unsigned char, std::allocator<unsigned char>>& vector)
{
    std::stringstream ss;
    for (auto item : vector)
    {
        ss << "0x";
        ss << std::hex << (int)item;
        ss << " ";
    }

    return ss.str();
}

void rev_func(const uxa_serial_msgs::msg::Transmit::ConstSharedPtr &msg)
{
    if (!status) 
    {
        RCLCPP_INFO(node->get_logger(), "STATUS: %d, DISCARDING MSG : %s", status, print_vector(msg->tx_data).c_str());
        return;
    }
        
    for (std::size_t i  = 0; i < msg->tx_data.size(); ++i) {
        msg_buf[i] = msg->tx_data[i];
    }
    RCLCPP_INFO(node->get_logger(), "RECIEVED SERIAL MSG : %s", print_vector(msg->tx_data).c_str());

    Send_Serial_String(SerialMode, msg_buf, msg->tx_data.size());
}


void handleStatusServiceRequest(
    const uxa_serial_msgs::srv::Status::Request::SharedPtr request, 
    uxa_serial_msgs::srv::Status::Response::SharedPtr response
) {
    RCLCPP_INFO(node->get_logger(), "NODE_STATUS_REQUEST, CURRENT STATUS: %d", status);
    response->status = status;
}

void handleSerialServiceRequest(
    const uxa_serial_msgs::srv::Serial::Request::SharedPtr request, 
    uxa_serial_msgs::srv::Serial::Response::SharedPtr response
) {
    for (std::size_t i  = 0; i < request->tx_data.size(); ++i) {
        msg_buf[i] = request->tx_data[i];
    }

    RCLCPP_INFO(node->get_logger(), "RECIEVED SERIAL MSG : %s, TRANSMITTING", print_vector(request->tx_data).c_str());

    Send_Serial_String(SerialMode, msg_buf, request->tx_data.size());

    auto total = request->expect * 2;
    unsigned char *receive_buf = new unsigned char[total * 2];

    if (request->expect > 0) {
        if(Read_Chars(SerialMode, receive_buf, total) == 1)
        {
            for (auto i = 0; i < total; i += 1)
                response->rx_data.push_back(receive_buf[i]);
            
            response->success = true;
            RCLCPP_INFO(node->get_logger(), "RECEIVED SERIAL RESPONSE: %s, SETTING RESPONSE", print_vector(response->rx_data).c_str());
        } else {
            RCLCPP_ERROR(node->get_logger(), "ERROR, FAILED TO RECEIVE SERIAL RESPONSE. SENDING 0xFF");
            response->success = false;
        }
    }

    delete []receive_buf;
}