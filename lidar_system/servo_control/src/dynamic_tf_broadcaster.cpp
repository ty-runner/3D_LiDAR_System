/*#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>

class DynamicTFBroadcaster : public rclcpp::Node
{
public:
  DynamicTFBroadcaster()
  : Node("dynamic_tf_broadcaster"), current_pan_(0.0), current_tilt_(0.0)
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/servo_angles", 10,
      std::bind(&DynamicTFBroadcaster::angle_callback, this, std::placeholders::_1));
  }

private:
  void angle_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 2) {
      RCLCPP_WARN(this->get_logger(), "Received servo_angles message with insufficient data");
      return;
    }
    current_pan_ = msg->data[0];
    current_tilt_ = msg->data[1];
    broadcast_transform();
  }

  void broadcast_transform()
  {
    auto t = geometry_msgs::msg::TransformStamped();
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "base_link";
    t.child_frame_id = "laser_frame";

    // No translation offset (adjust if needed)
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;

    // Convert angles from degrees to radians
    double pan_rad = current_pan_ * M_PI / 180.0;
    double tilt_rad = current_tilt_ * M_PI / 180.0;

    tf2::Quaternion q;
    // Use roll = 0, pitch = tilt, yaw = pan
    q.setRPY(0, tilt_rad, pan_rad);
    q.normalize();

    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
  double current_pan_;
  double current_tilt_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DynamicTFBroadcaster>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <cmath>

class DynamicTFBroadcaster : public rclcpp::Node
{
public:
  DynamicTFBroadcaster()
  : Node("dynamic_tf_broadcaster"),
    have_pose_(false),
    pan_deg_(0.0), tilt_deg_(0.0)
  {
    // Params
    parent_frame_ = this->declare_parameter<std::string>("parent_frame", "base_link"); // or "map"
    child_frame_  = this->declare_parameter<std::string>("child_frame",  "laser_frame");
    // Fixed offset of the LiDAR relative to parent_frame (meters)
    offset_x_ = this->declare_parameter<double>("offset_x", 0.0);
    offset_y_ = this->declare_parameter<double>("offset_y", 0.0);
    offset_z_ = this->declare_parameter<double>("offset_z", 0.0);
    // Publish rate (Hz)
    double pub_rate = this->declare_parameter<double>("publish_rate", 30.0);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Subscriptions
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
      std::bind(&DynamicTFBroadcaster::pose_cb, this, std::placeholders::_1));

    angles_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/servo_angles", 10,
      std::bind(&DynamicTFBroadcaster::angles_cb, this, std::placeholders::_1));

    // Timer to publish TF at a steady rate
    using namespace std::chrono_literals;
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / pub_rate)),
      std::bind(&DynamicTFBroadcaster::publish_tf, this));

    RCLCPP_INFO(get_logger(), "DynamicTFBroadcaster started. parent='%s' child='%s'",
                parent_frame_.c_str(), child_frame_.c_str());
  }

private:
  void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Translation from PX4 (already ENU via MAVROS)
    px_ = msg->pose.position.x;
    py_ = msg->pose.position.y;
    pz_ = msg->pose.position.z;
    last_stamp_ = msg->header.stamp;
    have_pose_ = true;
  }

  void angles_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 2) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "servo_angles has <2 values");
      return;
    }
    pan_deg_  = msg->data[0];
    tilt_deg_ = msg->data[1];
  }

  void publish_tf()
  {
    if (!have_pose_) {
      //  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      //                       "Waiting for /mavros/local_position/pose...");
      return;
    }

    geometry_msgs::msg::TransformStamped t;
    // Use the PX4 timestamp if available; fall back to now() if unset
    if (last_stamp_.seconds() == 0 && last_stamp_.nanoseconds() == 0) {
      t.header.stamp = this->get_clock()->now();
    } else {
      t.header.stamp = last_stamp_;
    }
    t.header.frame_id = parent_frame_;
    t.child_frame_id  = child_frame_;

    // Position = vehicle position + fixed sensor offset in parent frame
    t.transform.translation.x = px_ + offset_x_;
    t.transform.translation.y = py_ + offset_y_;
    t.transform.translation.z = pz_ + offset_z_;

    // Orientation from pan/tilt (degrees → radians); roll=0, pitch=tilt, yaw=pan
    const double pan_rad  = pan_deg_  * M_PI / 180.0;
    const double tilt_rad = tilt_deg_ * M_PI / 180.0;
    tf2::Quaternion q;
    q.setRPY(0.0, tilt_rad, pan_rad);
    q.normalize();

    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
  }

  // TF
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Subs
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr angles_sub_;

  // Latest data
  bool have_pose_;
  double px_{0.0}, py_{0.0}, pz_{0.0};
  rclcpp::Time last_stamp_{0,0,RCL_ROS_TIME};
  double pan_deg_, tilt_deg_;

  // Params
  std::string parent_frame_, child_frame_;
  double offset_x_, offset_y_, offset_z_;
};
 
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicTFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
