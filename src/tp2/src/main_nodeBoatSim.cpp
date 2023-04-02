#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <eigen3/Eigen/Dense>
#include <cmath>

// Définition des Quaternions
#include "tf2/LinearMath/Quaternion.h"
// Fonctions pour passer des Quaternions aux messages
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;
using namespace Eigen;
using std::placeholders::_1;

class BoatSimulator : public rclcpp::Node
{
public:
    BoatSimulator() : Node("boat_simulator")
    {
        timer_ = this->create_wall_timer(100ms, std::bind(&BoatSimulator::timer_callback, this));
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_publisher", 10);
        command_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("command_subscriber", 10,
                                                                                     std::bind(&BoatSimulator::set_u1, this, _1));
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("boat_marker", 10);
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = x(0);
        pose_msg.pose.position.y = x(1);
        pose_msg.pose.position.z = 0.0;
        double yaw = x(2);
        tf2::Quaternion q;
        q.setRPY(0., 0., yaw);
        pose_msg.pose.orientation = tf2::toMsg(q);

        // Publish the PoseStamped message on the desired topic
        RCLCPP_INFO(this->get_logger(), "Publishing pose: %f, %f, %f", pose_msg.pose.position.x, pose_msg.pose.position.y, yaw);
        pose_publisher_->publish(pose_msg);
        euler(x, u1, 0.1);
    }

    void euler(Matrix<double, 3, 1> &x, double u1, double dt)
    {
        Matrix<double, 3, 1> dx_ = Matrix<double, 3, 1>::Zero();
        dx_(0) = cos(x(2));
        dx_(1) = sin(x(2));
        dx_(2) = u1;
        x = x + dx_ * dt;
    }

    void set_u1(geometry_msgs::msg::Twist::SharedPtr msg)
    {
        u1 = msg->angular.z;
    }

    void init_parameters()
    {
        this->declare_parameter<double>("x0", 0.0);
        this->declare_parameter<double>("y0", 0.0);
        this->declare_parameter<double>("theta0", 0.0);
    }

    void publishBoatModelMarker(geometry_msgs::msg::PoseStamped pose_msg)
    {
        // Create the marker message
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "boat";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.mesh_resource = "package://tp2/meshes/boat.dae";
        marker.pose.position.x = pose_msg.pose.position.x;
        marker.pose.position.y = pose_msg.pose.position.y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = pose_msg.pose.orientation.x;
        marker.pose.orientation.y = pose_msg.pose.orientation.y;
        marker.pose.orientation.z = pose_msg.pose.orientation.z;
        marker.pose.orientation.w = pose_msg.pose.orientation.w;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;

        // Publish the marker message
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher =
            this->create_publisher<visualization_msgs::msg::Marker>("boat_marker", 10);
        publisher->publish(marker);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;

    Matrix<double, 3, 1> x = Matrix<double, 3, 1>::Zero();
    double u1 = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<BoatSimulator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
