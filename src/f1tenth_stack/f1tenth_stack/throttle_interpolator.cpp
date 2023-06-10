#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class ThrottleInterpolator : public rclcpp::Node
{
public:
  ThrottleInterpolator() : Node("throttle_interpolator")
  {
    this->declare_parameter("rpm_input_topic", "commands/motor/unsmoothed_speed");
    this->declare_parameter("rpm_output_topic", "commands/motor/speed");
    this->declare_parameter("servo_input_topic", "commands/servo/unsmoothed_position");
    this->declare_parameter("servo_output_topic", "commands/servo/position");
    this->declare_parameter("max_acceleration", 0.0);
    this->declare_parameter("speed_max", 0.0);
    this->declare_parameter("speed_min", 0.0);
    this->declare_parameter("throttle_smoother_rate", 0.0);
    this->declare_parameter("speed_to_erpm_gain", 0.0);
    this->declare_parameter("max_servo_speed", 0.0);
    this->declare_parameter("steering_angle_to_servo_gain", 0.0);
    this->declare_parameter("servo_smoother_rate", 0.0);
    this->declare_parameter("servo_max", 0.0);
    this->declare_parameter("servo_min", 0.0);
    this->declare_parameter("steering_angle_to_servo_offset", 0.0);

    this->get_parameter("rpm_input_topic", rpm_input_topic);
    this->get_parameter("rpm_output_topic", rpm_output_topic);
    this->get_parameter("servo_input_topic", servo_input_topic);
    this->get_parameter("servo_output_topic", servo_output_topic);
    this->get_parameter("max_acceleration", max_acceleration);
    this->get_parameter("speed_max", max_rpm);
    this->get_parameter("speed_min", min_rpm);
    this->get_parameter("throttle_smoother_rate", throttle_smoother_rate);
    this->get_parameter("speed_to_erpm_gain", speed_to_erpm_gain);
    this->get_parameter("max_servo_speed", max_servo_speed);
    this->get_parameter("steering_angle_to_servo_gain", steering_angle_to_servo_gain);
    this->get_parameter("servo_smoother_rate", servo_smoother_rate);
    this->get_parameter("servo_max", max_servo);
    this->get_parameter("servo_min", min_servo);
    this->get_parameter("steering_angle_to_servo_offset", last_servo);

    last_rpm = 0;
    desired_rpm = last_rpm;
    desired_servo_position = last_servo;

    rpm_output = this->create_publisher<std_msgs::msg::Float64>(rpm_output_topic, 1);
    servo_output = this->create_publisher<std_msgs::msg::Float64>(servo_output_topic, 1);

    rpm_sub = this->create_subscription<std_msgs::msg::Float64>(rpm_input_topic, 1, std::bind(&ThrottleInterpolator::_process_throttle_command, this, std::placeholders::_1));
    servo_sub = this->create_subscription<std_msgs::msg::Float64>(servo_input_topic, 1, std::bind(&ThrottleInterpolator::_process_servo_command, this, std::placeholders::_1));

    max_delta_servo = abs(steering_angle_to_servo_gain * max_servo_speed / servo_smoother_rate);
    servo_timer = this->create_wall_timer(std::chrono::duration<double>(1.0 / servo_smoother_rate), std::bind(&ThrottleInterpolator::_publish_servo_command, this));

    max_delta_rpm = abs(speed_to_erpm_gain * max_acceleration / throttle_smoother_rate);
    rmp_timer = this->create_wall_timer(std::chrono::duration<double>(1.0 / throttle_smoother_rate), std::bind(&ThrottleInterpolator::_publish_throttle_command, this));
  }

private:
  void _publish_throttle_command()
  {
    double desired_delta = desired_rpm - last_rpm;
    double clipped_delta = std::max(std::min(desired_delta, max_delta_rpm), -max_delta_rpm);
    double smoothed_rpm = last_rpm + clipped_delta;
    last_rpm = smoothed_rpm;
    // auto rpm_msg = std_msgs::msg::Float64();
    std_msgs::msg::Float64 rpm_msg;
    rpm_msg.data = static_cast<float>(smoothed_rpm);
    rpm_output->publish(rpm_msg);
  }

  void _process_throttle_command(const std_msgs::msg::Float64::SharedPtr msg)
  {
    double input_rpm = msg->data;
    // Do some sanity clipping
    input_rpm = std::min(std::max(input_rpm, min_rpm), max_rpm);
    desired_rpm = input_rpm;
  }

  void _publish_servo_command()
  {
    double desired_delta = desired_servo_position - last_servo;
    double clipped_delta = std::max(std::min(desired_delta, max_delta_servo), -max_delta_servo);
    double smoothed_servo = last_servo + clipped_delta;
    last_servo = smoothed_servo;
    // auto servo_msg = std::make_shared<std_msgs::msg::Float64>();
    std_msgs::msg::Float64 servo_msg;
    servo_msg.data = static_cast<float>(smoothed_servo);
    servo_output->publish(servo_msg);
  }

  void _process_servo_command(const std_msgs::msg::Float64::SharedPtr msg)
  {
    double input_servo = msg->data;
    // Do some sanity clipping
    input_servo = std::min(std::max(input_servo, min_servo), max_servo);
    // set the target servo position
    desired_servo_position = input_servo;
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rpm_output;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr servo_output;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rpm_sub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr servo_sub;
  rclcpp::TimerBase::SharedPtr servo_timer;
  rclcpp::TimerBase::SharedPtr rmp_timer;

  std::string rpm_input_topic;
  std::string rpm_output_topic;
  std::string servo_input_topic;
  std::string servo_output_topic;
  double max_acceleration;
  double max_rpm;
  double min_rpm;
  double throttle_smoother_rate;
  double speed_to_erpm_gain;
  double max_servo_speed;
  double steering_angle_to_servo_gain;
  double servo_smoother_rate;
  double max_servo;
  double min_servo;
  double last_servo;

  double last_rpm;
  double desired_rpm;
  double desired_servo_position;
  double max_delta_servo;
  double max_delta_rpm;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto p = std::make_shared<ThrottleInterpolator>();
  rclcpp::spin(p);
  rclcpp::shutdown();
  return 0;
}