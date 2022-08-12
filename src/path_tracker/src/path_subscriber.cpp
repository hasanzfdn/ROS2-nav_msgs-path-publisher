#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <iostream>
#include <vector>
#include <unistd.h>


using std::placeholders::_1;

class PathSubscriber : public rclcpp::Node
{
public:
    PathSubscriber() : Node("Path_Subscriber")
  {
      publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("Vehicle_polygon", 10);
      subscription_ = this->create_subscription<nav_msgs::msg::Path>("path_ellipse_partial",10, std::bind(&PathSubscriber::topic_callback, this, _1));
  }


private:
  void topic_callback(const nav_msgs::msg::Path::ConstSharedPtr msg)
  {
    if (path_message.empty())
    {
        path_message = msg->poses;
    }
      for (long unsigned int i = 0; i < path_message.size(); ++i) {
           geometry_msgs::msg::Point32 point_;
           vehicle.polygon.points.clear();
           point_.x=path_message[i].pose.position.x+(0.5);
           point_.y=path_message[i].pose.position.y-(0.5);
           point_.z=path_message[i].pose.position.z+(0.5);
           vehicle.polygon.points.push_back(point_);

           point_.x=path_message[i].pose.position.x-(0.5);
           point_.y=path_message[i].pose.position.y-(0.5);
           point_.z=path_message[i].pose.position.z+(0.5);
          vehicle.polygon.points.push_back(point_);

           point_.x=path_message[i].pose.position.x-(0.5);
           point_.y=path_message[i].pose.position.y+(1.5);
           point_.z=path_message[i].pose.position.z+(0.5);
          vehicle.polygon.points.push_back(point_);

           point_.x=path_message[i].pose.position.x+(0.5);
           point_.y=path_message[i].pose.position.y+(1.5);
           point_.z=path_message[i].pose.position.z+(0.5);
          vehicle.polygon.points.push_back(point_);
          //Hz
          sleep(0.8);

          vehicle.header.frame_id = "map";
          publisher_->publish(vehicle);
      }


  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr publisher_;

  std::vector<geometry_msgs::msg::PoseStamped> path_message;
  geometry_msgs::msg::PolygonStamped vehicle;
  rclcpp::TimerBase::SharedPtr Hz;

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathSubscriber>());
  rclcpp::shutdown();
  return 0;
}
