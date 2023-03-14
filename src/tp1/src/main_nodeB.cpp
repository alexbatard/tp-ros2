#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber() : Node("minimal_subscriber")
    {
        // Crée un subscriber sur le topic "heading" de type Float64 qui appèlera la fonction
        // topic_callback pour chaque nouveau message
        subscription_ = this->create_subscription<std_msgs::msg::Float64>("heading", 10,
                                                                          std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

private:
    // callback function
    void topic_callback(const std_msgs::msg::Float64 &msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg.data);
    }

    // Objet de subscription, membre de la classe
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<MinimalSubscriber>();
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client = node->create_client<std_srvs::srv::Trigger>("boat_info");

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    while (!client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);

    // Wait for the result
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Boat info: %s", result.get()->message.c_str());
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service boat_info");
    }

    // rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
