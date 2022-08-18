
#include "path_subscriber.hpp"


PathSubscriber::PathSubscriber()
    : Node("tracker")
  {
      this->declare_parameter("frequency", 0.1);
      this->declare_parameter("car_length", 2.0);
      this->declare_parameter("car_width", 1.0);
      this->declare_parameter("rear_overhang", 0.5);
      this->declare_parameter("path_topic", "/path_ellipse_partial");
      this->declare_parameter("publish_vehicle_topic", "/Vehicle_polygon");
      this->declare_parameter("base_link_topic", "/base_link_topic");


      this->get_parameter("frequency", frequency);
      this->get_parameter("rear_overhang", rear_overhang);
      this->get_parameter("car_length", car_length);
      this->get_parameter("car_width", car_width);
      this->get_parameter("path_topic", path_topic);
      this->get_parameter("publish_vehicle_topic", publish_vehicle_topic);
      this->get_parameter("base_link_topic", base_link_topic);


      RCLCPP_INFO(this->get_logger(), "----------------------------------------------------");
      RCLCPP_INFO(this->get_logger(), "frequency: %f", frequency);
      RCLCPP_INFO(this->get_logger(), "car_length: %f", car_length);
      RCLCPP_INFO(this->get_logger(), "car_width: %f", car_width);
      RCLCPP_INFO(this->get_logger(), "rear_overhang: %f", rear_overhang);
      RCLCPP_INFO(this->get_logger(), "path_topic: %s", path_topic.c_str());
      RCLCPP_INFO(this->get_logger(), "publish_vehicle_topic: %s", publish_vehicle_topic.c_str());
      RCLCPP_INFO(this->get_logger(), "base_link_topic: %s", base_link_topic.c_str());
      RCLCPP_INFO(this->get_logger(), "----------------------------------------------------");

      publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(publish_vehicle_topic, 10);
      base_link_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(base_link_topic, 10);
      subscription_ = this->create_subscription<nav_msgs::msg::Path>(path_topic,10, std::bind(&PathSubscriber::topic_callback, this, _1));
      timer_ = this->create_wall_timer(frequency*1000ms, std::bind(&PathSubscriber::create_polygon_points, this));
      }


    geometry_msgs::msg::PolygonStamped PathSubscriber::rotate_point (geometry_msgs::msg::PoseStamped pose, geometry_msgs::msg::PolygonStamped vehicle){
     double yaw,pitch,roll;

     // Convert quaternion values to yaw,pitch,roll degree.
     tf2::Quaternion q(pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w);
     tf2::Matrix3x3 q_matrix(q);
     q_matrix.getEulerYPR(yaw,pitch,roll);

     // Define polygon variable.
     geometry_msgs::msg::PolygonStamped temp_vehicle;

     // Rotate each polygon points by using yaw angle.
     for (auto point : vehicle.polygon.points){

         // Rotate polygon about base_link.
         complex_point base_link(pose.pose.position.x, pose.pose.position.y);
         complex_point polygon(point.x, point.y);

         // Compute x and y coordinates of new polygon point.
         complex_point New_polygon_point=(polygon-base_link) * polar(1.0, yaw) + base_link;

         // Assign x and y values to temp_vehicle variable.
         geometry_msgs::msg::Point32 point_;
         point_.x = New_polygon_point.real();
         point_.y = New_polygon_point.imag();
         temp_vehicle.polygon.points.push_back(point_);
     }
        return temp_vehicle;
    }

   void PathSubscriber::create_polygon_points(){
        geometry_msgs::msg::Point32 point_;

       std::cout << "create polygon points " << std::endl;

       if(path_message.empty()){
           return;
       }

        if (counter <= path_message.size()) {

            // Create polygon points according to base_link points.( Base_link almost center of the car).
            vehicle.polygon.points.clear();
            point_.x = path_message[counter].pose.position.x + (car_length - rear_overhang);
            point_.y = path_message[counter].pose.position.y - (car_width / 2);

            // Fill the vehicle polygon variable with each created point.
            vehicle.polygon.points.push_back(point_);


            point_.x = path_message[counter].pose.position.x - (rear_overhang);
            point_.y = path_message[counter].pose.position.y - (car_width / 2);
            vehicle.polygon.points.push_back(point_);

            point_.x = path_message[counter].pose.position.x - (rear_overhang);
            point_.y = path_message[counter].pose.position.y + (car_width / 2);
            vehicle.polygon.points.push_back(point_);

            point_.x = path_message[counter].pose.position.x + (car_length - rear_overhang);
            point_.y = path_message[counter].pose.position.y + (car_width / 2);

            vehicle.polygon.points.push_back(point_);

            base_link = path_message[counter];

            vehicle =  rotate_point(path_message[counter],vehicle);
            vehicle.header.frame_id = "map";
            base_link.header.frame_id = "map";
            publisher_->publish(vehicle);
            base_link_publisher_->publish(base_link);
            std::cout << "published vehicle." << std::endl;

            counter++;

        }
        else {
            counter = 0;
        }

    }


  void PathSubscriber::topic_callback(const nav_msgs::msg::Path::ConstSharedPtr msg) {

      // Get path message for once.
    if (path_message.empty())
    {
        // Assign poses values to path message which comes from topic.
        path_message = msg->poses;
    }

  }


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathSubscriber>());
  rclcpp::shutdown();
  return 0;
}
