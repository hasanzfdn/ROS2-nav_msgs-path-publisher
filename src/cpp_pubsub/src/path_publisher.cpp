// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
      double pi = 3.14159265359;
      radian = theta_ * (pi / 180);
      x = 15 * cos(radian);
      y = 0.5 * 15 * sin(radian);

      // Fill current path point to a temporary variable.
      geometry_msgs::msg::PoseStamped tempPoint;
      tempPoint.pose.position.set__x(x);
      tempPoint.pose.position.set__y(y);
      tempPoint.pose.position.set__z(z);

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
