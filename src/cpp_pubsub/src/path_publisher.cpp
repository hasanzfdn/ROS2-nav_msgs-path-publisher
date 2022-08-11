#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <iostream>
#include <math.h>

using namespace std::chrono_literals;

class Path_publisher : public rclcpp::Node
{
public:
  Path_publisher() : Node("Path_publisher"), count_(0)
  {   


  // Declare variables for path generation.
  float x,y, z = 0;
  double radian;
  for (int theta_=0; theta_<= 270; theta_=theta_+2)
    {
      // Compute current path point.
      radian = theta_ * (M_PI / 180);
      x = 15 * cos(radian);
      y = 0.5 * 15 * sin(radian);

      // Fill current path point to a temporary variable.
      geometry_msgs::msg::PoseStamped tempPoint;
      tempPoint.pose.position.x=x;
      tempPoint.pose.position.y=y;
      tempPoint.pose.position.z=z;

      // Add point to path.
      path_message.header.frame_id = "map";
      path_message.poses.push_back(tempPoint);

    }
    
    // Create publisher and timer.
    publisher_ = this->create_publisher<nav_msgs::msg::Path>("path_ellipse_partial", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&Path_publisher::timer_callback, this));

  }

private:
  void timer_callback()
  {
    // Publish path.
    RCLCPP_INFO(this->get_logger(), "Publishing path, path size: %d",static_cast<int>(path_message.poses.size()));
    publisher_->publish(path_message);
  }

  // Declare variables.
  nav_msgs::msg::Path path_message;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;


};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Path_publisher>());
  rclcpp::shutdown();
  return 0;
}
