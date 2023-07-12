#include <telemetry/telemetry_sender.hpp>


Telemetry::Telemetry() : Node("Telemetry")
{

  if ((this->client_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      RCLCPP_DEBUG(this->get_logger(), "\n Socket creation error \n");
    }

  serv_ip.sin_family = AF_INET;
  serv_ip.sin_addr.s_addr = inet_addr(this->ip_address);
  serv_ip.sin_port = htons(this->port_number);
  

  RCLCPP_DEBUG(this->get_logger(), "Node Started\n");
  //Subscribers to get the data from the car
  this->autonomy_enabled_sub_ = this->create_subscription<std_msgs::msg::Bool>("autonomy_enabled", 1, std::bind(&Telemetry::auto_callback, this, std::placeholders::_1));
  this->rpm_sub_ = this->create_subscription<std_msgs::msg::Float64>("commands/motor/speed", 1, std::bind(&Telemetry::rpm_callback, this, std::placeholders::_1));
  this->servo_pos_sub_ = this->create_subscription<std_msgs::msg::Float64>("commands/servo/position", 1, std::bind(&Telemetry::servo_callback, this, std::placeholders::_1));

  packet_timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Telemetry::send_packet, this)); //Timer to send the UDP packet
}

void Telemetry::send_packet(void){

  char packet_payload[sizeof(this->packet1)]; //creates byte array to hold the packet contents

  memcpy(&packet_payload, &(this->packet1), sizeof(this->packet1));

  sendto(this->client_fd, packet_payload, sizeof(packet_payload), 0, reinterpret_cast<sockaddr*>(&(this->serv_ip)), sizeof(this->serv_ip)); //sends the byte array
  RCLCPP_DEBUG(this->get_logger(), "Packet Sent\n");
}

void Telemetry::auto_callback(const std_msgs::msg::Bool::SharedPtr auto_enabled){
  this->packet1.current_auto_enabled_ = auto_enabled->data;
}
    
void Telemetry::rpm_callback(const std_msgs::msg::Float64::SharedPtr rpm){
  this->packet1.current_rpm_ = rpm->data;
}

void Telemetry::servo_callback(const std_msgs::msg::Float64::SharedPtr position){
  this->packet1.current_servo_position_ = position->data;
}



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Telemetry>());
  rclcpp::shutdown();
  return 0;
}
