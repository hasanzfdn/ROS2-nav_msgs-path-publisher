// Created by hasan on 18.08.2022.

#include <functional>
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <unistd.h>
#include <complex.h>
#include "rcl_yaml_param_parser/parser.h"

using std::placeholders::_1;
using namespace std;
typedef complex<double> complex_point;

    class PathSubscriber : public rclcpp::Node
    {
    public:

        PathSubscriber();
    private:

   //     bool readParameters();

        geometry_msgs::msg::PolygonStamped rotate_point (geometry_msgs::msg::PoseStamped pose, geometry_msgs::msg::PolygonStamped vehicle);
        void create_polygon_points();
        void topic_callback(const nav_msgs::msg::Path::ConstSharedPtr msg);

        size_t counter = 0;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_;
        rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr publisher_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr base_link_publisher_;
        std::vector<geometry_msgs::msg::PoseStamped> path_message;
        geometry_msgs::msg::PoseStamped base_link;
        geometry_msgs::msg::PolygonStamped vehicle;
        double frequency,car_length, car_width, rear_overhang;
        std::string path_topic,publish_vehicle_topic, base_link_topic;
        rclcpp::TimerBase::SharedPtr timer_;

    };


