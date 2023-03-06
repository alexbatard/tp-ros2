#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

// Namespace used for bindings (_1, _2, etc.)
using std::placeholders::_1;
// or "using namespace std::placeholders;"

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
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}