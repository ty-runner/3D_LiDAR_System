#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"

class LaserToPointCloudNode : public rclcpp::Node
{
public:
  LaserToPointCloudNode()
  : Node("lidar_pointcloud_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&LaserToPointCloudNode::scan_callback, this, std::placeholders::_1));

    pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_points", 10);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    sensor_msgs::msg::PointCloud2 cloud_out;
    try {
      // Project the laser scan into a point cloud in the scan's frame.
      projector_.projectLaser(*scan_msg, cloud_out);
      
      // Lookup transform from scan frame to "base_link".
      auto transform = tf_buffer_.lookupTransform(
        "base_link", scan_msg->header.frame_id, scan_msg->header.stamp,
        rclcpp::Duration::from_seconds(1.0));

      sensor_msgs::msg::PointCloud2 cloud_transformed;
      tf2::doTransform(cloud_out, cloud_transformed, transform);
      cloud_transformed.header.frame_id = "base_link";

      pointcloud_publisher_->publish(cloud_transformed);
      RCLCPP_INFO(this->get_logger(), "Published PointCloud2 data");
    }
    catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not transform scan: %s", ex.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
  laser_geometry::LaserProjection projector_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaserToPointCloudNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
