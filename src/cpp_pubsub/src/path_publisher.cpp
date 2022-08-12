#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <math.h>
#include <Eigen/Dense>

using namespace std::chrono_literals;
using namespace Eigen; 

class Path_publisher : public rclcpp::Node
{
public:
  Path_publisher() : Node("Path_publisher"), count_(0)
  {   

  // Declare variables for path generation.
  float x,y, z = 0;
  double radian;
  Quaternionf q;
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


      if (!path_message.poses.empty())
      {
        
        // Calculate yaw angle between two points.
        float b=x-path_message.poses.back().pose.position.x;
        float a=y-path_message.poses.back().pose.position.y;
        double yaw = atan2(a, b);

        // Transform to quaternion angles.
        float roll = 0, pitch = 0;    
        q = AngleAxisf(roll, Vector3f::UnitX())
        * AngleAxisf(pitch, Vector3f::UnitY())
        * AngleAxisf(yaw, Vector3f::UnitZ());
        
        // Set orientation value to path points.
        path_message.poses.back().pose.orientation.x = q.x();
        path_message.poses.back().pose.orientation.y = q.y();
        path_message.poses.back().pose.orientation.z = q.z();
        path_message.poses.back().pose.orientation.w = q.w();
      
      }
    
      // Add points to path.
      path_message.header.frame_id = "map";
      path_message.poses.push_back(tempPoint);

    }

    // Set orientation to the last path point.
    path_message.poses.back().pose.orientation.x = q.x();
    path_message.poses.back().pose.orientation.y = q.y();
    path_message.poses.back().pose.orientation.z = q.z();
    path_message.poses.back().pose.orientation.w = q.w();
  
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
