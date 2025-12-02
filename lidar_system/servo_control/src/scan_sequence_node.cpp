#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <algorithm>

class ScanSequenceNode : public rclcpp::Node
{
public:
  ScanSequenceNode()
  : Node("scan_sequence_node"), pan_angle_(0.0), tilt_angle_(150.0),
    pan_increment_(0.75), pan_direction_(1)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/servo_angles", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(5),  // 10Hz update rate
      std::bind(&ScanSequenceNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
  auto msg = std_msgs::msg::Float32MultiArray();
  msg.data.push_back(pan_angle_);
  msg.data.push_back(tilt_angle_);
  publisher_->publish(msg);

  pan_angle_ += pan_increment_ * pan_direction_;

  if (pan_angle_ >= 180.0 || pan_angle_ <= 0.0) {
    pan_direction_ *= -1;
    pan_angle_ = std::max(std::min(pan_angle_, 180.0), 0.0);
    tilt_angle_ += 5.0; // Increment tilt by 5 degrees each sweep

    if (tilt_angle_ > 180.0) { // Set the upper limit as 200 degrees
      tilt_angle_ = 125.0; // Reset to lower limit was 105
    }
    RCLCPP_INFO(this->get_logger(), "Pan reversed at %.2f°, Tilt updated to %.2f°", pan_angle_, tilt_angle_);
    }
  }

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  double pan_angle_;
  double tilt_angle_;
  double pan_increment_;
  int pan_direction_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScanSequenceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
