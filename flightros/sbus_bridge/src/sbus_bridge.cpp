#include "sbus_bridge/sbus_bridge.h"

namespace sbus_bridge{

SbusBridge::SbusBridge()
: Node("sbus_bridge")
    {
        // Configuration file dictionary
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "This parameter is configuration file dictionary";
        this->declare_parameter("sbus_bridge_cfg_path", "$(find sbus_bridge)/configs/default.yaml", param_desc);

        // Load parameters
        loadParameters();

        //
        serialPortReceiveThread(port_name_);

        // Subscriber
        control_command_sub_ = this->create_subscription<msgs::msg::ControlCommand>("control_command", 1, std::bind(&SbusBridge::controlCommandCallback, this, _1));
    }

SbusBridge::~SbusBridge()
{
    // 
}

void SbusBridge::startReceiverThread(const std::string& port)
{
    // Open serial port; O_RDWR - Read and write; O_NOCTTY - Ignore special chars like CTRL-C
    serial_port_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
    RCLCPP_INFO(this->get_logger(), "[%s] Connect to serial port", this->get_namespace());
    if (serial_port_fd_ == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "[%s] Could not open serial port %s", this->get_namespace(), port.c_str());
    }

    // Configure serial port for sbus
    // clear config
    fcntl(serial_port_fd_, F_SETFL, 0);
    // read non blocking
    fcntl(serial_port_fd_, F_SETFL, FNDELAY);
    struct termios2 uart_config;
    /* Fill the struct for the new configuration */
    ioctl(serial_port_fd_, TCGETS2, &uart_config);
    // Output flags - Turn off output processing
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    uart_config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
    // Input flags - Turn off input processing
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    // No line processing:
    // echo off
    // echo newline off
    // canonical mode off,
    // extended input processing off
    // signal chars off
    uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    // Turn off character processing
    // Turn off odd parity
    uart_config.c_cflag &= ~(CSIZE | PARODD | CBAUD);
    // Enable parity generation on output and parity checking for input.
    uart_config.c_cflag |= PARENB;
    // Set two stop bits, rather than one.
    uart_config.c_cflag |= CSTOPB;
    // No output processing, force 8 bit input
    uart_config.c_cflag |= CS8;
    // Enable a non standard baud rate
    uart_config.c_cflag |= BOTHER;
    // Set custom baud rate of 100'000 bits/s necessary for sbus
    const speed_t spd = 100000;
    uart_config.c_ispeed = spd;
    uart_config.c_ospeed = spd;
    if (ioctl(serial_port_fd_, TCSETS2, &uart_config) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "[%s] could not set configuration of serial port", this->get_namespace());
    }

    // receive thread





}

void SbusBridge::loadParameters()
{
    // Load configuration file
    std::string cfg_path = this->get_parameter("sbus_bridge_cfg_path").as_string();
    YAML::Node cfg_ = YAML::LoadFile(cfg_path);
    
    quadrotor_mass_ = cfg_["mass"].as<double>();
    RCLCPP_INFO_STREAM(this->get_logger(), this->get_namespace()<< "->Parameter[quadrotor_mass]: "<< quadrotor_mass_);
   
    collective_thrust_map_a_ = cfg_["thrust_map_a"].as<double>();
    RCLCPP_INFO_STREAM(this->get_logger(), this->get_namespace()<< "->Parameter[collective_thrust_map_a]: "<< collective_thrust_map_a_);
    
    collective_thrust_map_b_ = cfg_["thrust_map_b"].as<double>();
    RCLCPP_INFO_STREAM(this->get_logger(), this->get_namespace()<< "->Parameter[collective_thrust_map_b]: "<< collective_thrust_map_b_);
    
    collective_thrust_map_c_ = cfg_["thrust_map_c"].as<double>();
    RCLCPP_INFO_STREAM(this->get_logger(), this->get_namespace()<< "->Parameter[collective_thrust_map_c]: "<< collective_thrust_map_c_);
    
    max_roll_rate_ = cfg_["max_roll_rate"].as<double>();
    RCLCPP_INFO_STREAM(this->get_logger(), this->get_namespace()<< "->Parameter[max_roll_rate]: "<< max_roll_rate_);
    max_roll_rate_ /= (180.0 / M_PI);// [rad/s]
    
    max_pitch_rate_ = cfg_["max_pitch_rate"].as<double>();
    RCLCPP_INFO_STREAM(this->get_logger(), this->get_namespace()<< "->Parameter[max_pitch_rate]: "<< max_pitch_rate_);
    max_pitch_rate_ /= (180.0 / M_PI);// [rad/s]
    
    max_yaw_rate_ = cfg_["max_yaw_rate"].as<double>();
    RCLCPP_INFO_STREAM(this->get_logger(), this->get_namespace()<< "->Parameter[max_yaw_rate]: "<< max_yaw_rate_);
    max_yaw_rate_ /= (180.0 / M_PI);// [rad/s]

    port_name_ = cfg_["port_name"].as<std::string>();
    RCLCPP_INFO_STREAM(this->get_logger(), this->get_namespace()<< "->Parameter[port_name]: "<< port_name_);
      
    return;
}

void SbusBridge::controlCommandCallback(const msgs::msg::ControlCommand::ConstPtr & received_control_command)
{
    // Generate sbus message from control command
    msgs::msg::Sbus sbus_msg;
    
    // Throttle
    //Citardauq Formula: Gives a numerically stable solution of the quadratic equation for collective_thrust_map_a ~ 0, which is not the case for the standard formula.
    sbus_msg.channels[sbus_msg.KTHROTTLE]  = round(2.0 * (collective_thrust_map_c_ - received_control_command->collective_thrust * quadrotor_mass_) /
     (-collective_thrust_map_b_ - sqrt(collective_thrust_map_b_ * collective_thrust_map_b_ - 4.0 * collective_thrust_map_a_ * (collective_thrust_map_c_ - received_control_command->collective_thrust * quadrotor_mass_))));
    
    // Roll
    sbus_msg.channels[sbus_msg.KROLL] = round((received_control_command->bodyrates.x / max_roll_rate_) * (msgs::msg::Sbus::KMAXCMD - msgs::msg::Sbus::KMEANCMD) + msgs::msg::Sbus::KMEANCMD);

    // Pitch
    sbus_msg.channels[sbus_msg.KPITCH] = round((received_control_command->bodyrates.y / max_roll_rate_) * (msgs::msg::Sbus::KMAXCMD - msgs::msg::Sbus::KMEANCMD) + msgs::msg::Sbus::KMEANCMD);

    // Yaw
    sbus_msg.channels[sbus_msg.KYAW] = round((received_control_command->bodyrates.z / max_roll_rate_) * (msgs::msg::Sbus::KMAXCMD - msgs::msg::Sbus::KMEANCMD) + msgs::msg::Sbus::KMEANCMD);

    // Connection state
    sbus_msg.channels[sbus_msg.KCONNECTIONSTATE] = sbus_msg.KMAXCMD;// 2000 to arm

    // Bridge state
    sbus_msg.channels[sbus_msg.KBRIDGESTATE] = sbus_msg.KMAXCMD;// 2000 to arm

    // Arm state
    sbus_msg.channels[sbus_msg.KARMSTATE] = sbus_msg.KMAXCMD;// 2000 to arm

    // Control mode
    sbus_msg.channels[sbus_msg.KCONTROLMODE] = sbus_msg.KMAXCMD;// 2000 to bodyrates control

    for (int i = (sbus_msg.KCONTROLMODE + 1); i < sbus_msg.KNCHANNELS; i++)
    {
        sbus_msg.channels[i] = sbus_msg.KMEANCMD;// 1500
    }

    sbus_msg.digital_channel_1 = false;

    sbus_msg.digital_channel_2 = false;

    sbus_msg.frame_lost = false;

    sbus_msg.failsafe = false;

    sbus_msg_from_control_command_ = sbus_msg;
    
    return;
}

}// namespace sbus_bridge