#ifndef TELEMETRY_HPP
#define TELEMETRY_HPP

#include <chrono>
#include <vector>
#include <map>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

class Telemetry : public rclcpp::Node
{
    public:
        Telemetry();

    private:

    void auto_callback(const std_msgs::msg::Bool::SharedPtr auto_enabled);
    void rpm_callback(const std_msgs::msg::Float64::SharedPtr rpm);
    void servo_callback(const std_msgs::msg::Float64::SharedPtr position);
    void send_packet(void);

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autonomy_enabled_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rpm_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr servo_pos_sub_;

    char * ip_address = "127.0.0.1";
    unsigned short int port_number = 10000;
    struct sockaddr_in serv_ip;
    int client_fd;
    rclcpp::TimerBase::SharedPtr packet_timer;  

    struct data_packet{
        bool current_auto_enabled_;
        float current_rpm_;
        float current_servo_position_;
    };
    
    struct data_packet packet1;
};


#endif //TELEMETRY_HPP
