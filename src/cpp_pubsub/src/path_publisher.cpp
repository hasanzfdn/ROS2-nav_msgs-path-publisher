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

using namespace std::chrono_literals;

class Path_publisher : public rclcpp::Node
{
public:
  Path_publisher() : Node("Path_publisher"), count_(0)
  {   

  while (theta_ <= 270)
    {
      x = 15 * cos(theta_);
      y = 0.5 * 15 * sin(theta_);

      geometry_msgs::msg::PoseStamped tempPoint;
      tempPoint.pose.position.set__x(x);
      tempPoint.pose.position.set__y(y);
      tempPoint.pose.position.set__z(z);
      message.header.frame_id = "map";
      message.poses.push_back(tempPoint);

      theta_ += 2;
    }
    
    std::cout<<message.poses.size();
    
    publisher_ = this->create_publisher<nav_msgs::msg::Path>("path_ellipse_partial", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&Path_publisher::timer_callback, this));
    publisher_2 = this->create_publisher<std_msgs::msg::String>("topic_name", 10);

  }

private:
  void timer_callback()
  {
    auto message2 = std_msgs::msg::String();
    message2.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message2.data.c_str());
    publisher_->publish(message);
    publisher_2->publish(message2);
  }

  nav_msgs::msg::Path message;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_2;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;

  int x, y;
  int theta_ = 0;
  int z = 0;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Path_publisher>());
  rclcpp::shutdown();
  return 0;
}
