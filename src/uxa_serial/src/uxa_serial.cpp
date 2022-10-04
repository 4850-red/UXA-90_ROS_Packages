#include "uxa_serial.h"

std::shared_ptr<rclcpp::Node> node;

int main(int argc, char **argv)
{
    sleep(10);

    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("uxa_serial");

    msg_buf = new unsigned char[_MSG_BUFF_SIZE];


    // ros::Publisher uxa_serial_pub = n.advertise<uxa_serial_msgs::receive>("uxa_serial_publisher", _MSG_BUFF_SIZE);
    // ros::Subscriber uxa_serial_sub = n.subscribe<uxa_serial_msgs::transmit>("uxa_serial_subscriber", _MSG_BUFF_SIZE, rev_func);

    auto uxa_serial_pub = node->create_publisher<uxa_serial_msgs::msg::Receive>("uxa_serial_publisher", _MSG_BUFF_SIZE);
    rclcpp::Rate loop_rate(1000);

    auto uxa_serial_sub = node->create_subscription<uxa_serial_msgs::msg::Transmit>("uxa_serial_subscriber", 10, rev_func);

    // ros::Rate loop_rate(1000);

    if((Serial = Init_Serial(_SERIAL_PORT)) != -1)
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

        Send_Serial_String(Serial, Trans_chr, cnt);
        sleep(1);

        cnt = 0;

        Trans_chr[cnt++] = 0xFF;
        Trans_chr[cnt++] = (unsigned char)(7 << 5);
        Trans_chr[cnt++] = 225;
        Trans_chr[cnt++] = 0;
        Trans_chr[cnt++] = 0X07;
        Trans_chr[cnt++] = (Trans_chr[1]^Trans_chr[2]^Trans_chr[3]^Trans_chr[4]) & 0x7F;

        Send_Serial_String(Serial, Trans_chr, cnt);
        sleep(1);

        memset(Trans_chr, '\0', sizeof(Trans_chr));
        memset(Recev_chr, '\0', sizeof(Recev_chr));

        cout << "SERIAL : " <<  "Serial communication stand by." << endl << endl;

        uxa_serial_msgs::msg::Receive msg;


        while(rclcpp::ok())
        {
            loop_rate.sleep();
            if(Read_Serial_Char(Serial, Recev_chr) == 1)
            {
                msg.rx_data = Recev_chr[0];
                uxa_serial_pub->publish(msg);
            }
            rclcpp::spin_some(node);
        }

    }

    cout << endl;
    cout << "SERIAL : " << Serial << " Device close." << endl;
    close(Serial);
    cout << "SERIAL : " << "uxa_serial node terminate." << endl;
    delete msg_buf;
    return 0;
}


int Init_Serial(const char *Serial_Port)
{

    termios Serial_Setting;

    if((Serial = open(Serial_Port, O_RDWR | O_NONBLOCK | O_NOCTTY)) == -1)
    {
        cout << "SERIAL : " << Serial_Port << " Device open error" << endl;
        cout << "SERIAL : " << Serial_Port << " Device permission change progress...." << endl;
        for(int temp = 0; temp < 5; temp++)
        {
            if(chmod(Serial_Port, __S_IREAD | __S_IWRITE) == 0){

                cout << "SERIAL : " << Serial_Port << " Device permission change complete" << endl;
                Serial = open(Serial_Port, O_RDWR | O_NONBLOCK | O_NOCTTY);

                if(Serial == -1)
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

    tcflush(Serial, TCIOFLUSH); // originally TCIFLUSH
    tcsetattr(Serial, TCSANOW, &Serial_Setting);

    return Serial;

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
    for (std::size_t i  = 0; i < msg->tx_data.size(); ++i) {
        msg_buf[i] = msg->tx_data[i];
    }
    RCLCPP_INFO(node->get_logger(), "receive msg : 0x%s", print_vector(msg->tx_data).c_str());

    Send_Serial_String(Serial, msg_buf, msg->tx_data.size());
}
