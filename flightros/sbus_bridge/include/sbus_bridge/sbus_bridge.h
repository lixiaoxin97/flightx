#pragma once

#include <fcntl.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>

#include "rclcpp/rclcpp.hpp"
#include "msgs/msg/control_command.hpp"
#include "msgs/msg/sbus.hpp"
#include "yaml-cpp/yaml.h"

using std::placeholders::_1;

namespace sbus_bridge{

class SbusBridge : public rclcpp::Node
{
  public:
    SbusBridge();
    virtual ~SbusBridge();

  protected:
    // Sbus serial port functions
    void startReceiverThread(const std::string& port);
    void serialPortReceiveThread(const std::string& port);

  private:
    // Load parameters
    void loadParameters();
    
    // Callback functions
    void controlCommandCallback(const msgs::msg::ControlCommand::ConstPtr & received_control_command);

    // Self-define functions
    //

    // Subscriber
    rclcpp::Subscription<msgs::msg::ControlCommand>::SharedPtr control_command_sub_;

    // Variables
    msgs::msg::Sbus sbus_msg_from_control_command_;
    msgs::msg::Sbus sbus_msg_from_remote_control_;
    msgs::msg::Sbus sbus_msg_to_send_;

    // Configuration parameters
    double quadrotor_mass_;
    double collective_thrust_map_a_;
    double collective_thrust_map_b_;
    double collective_thrust_map_c_;
    double max_roll_rate_;
    double max_pitch_rate_;
    double max_yaw_rate_;
    std::string port_name_;

    int serial_port_fd_;

    // Sbus constants
    //

};

}// namespace sbus_bridge