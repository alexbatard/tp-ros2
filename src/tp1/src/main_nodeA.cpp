#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"
// include each message type you want to use

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

/*
 * Classe qui hérite de l'objet rclcpp::Node.
 * Vous pouvez choisir de séparer votre classe dans un fichier .hpp et .cpp
 */
class MinimalPublisher : public rclcpp::Node
{
public:
    /* Constructeur de votre node avec le nom du node "minimal_publisher"
     */
    MinimalPublisher() : Node("minimal_publisher")
    {
        // Créer un publisher de type std_msgs/msg/float64 sur le topic "heading", avec
        // une liste d'attente de 10 messages maximum
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("heading", 10);
        // Créer un timer qui appelle la fonction time_callback toutes les 500ms
        timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
        // À noter qu'il existe plusieurs base de temps possible
        service_ = this->create_service<std_srvs::srv::Trigger>("boat_info", std::bind(&MinimalPublisher::handle_service_request, this, _1, _2));
    }

private:
    /* Fonction de callback du timer \
     */
    void timer_callback()
    {
        // Créer un object message de type float64
        auto message = std_msgs::msg::Float64();
        // Rempli le contenu du message
        message.data = 45.0 * sin(rclcpp::Clock().now().seconds());
        // Affiche un log dans la console (format de fprintf)
        // Il est également possible d'utiliser RCLCPP_WARN
        // En dehors d'un node, on peut utiliser rclcpp::get_logger("rclcpp")
        RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.data);
        // Publie le message en utilisation l'objet publisher
        publisher_->publish(message);
    }

    void handle_service_request(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Incoming request");
        response->message = "name: DD Boat 1, motor state: on";
    }
    rclcpp::TimerBase::SharedPtr timer_;                             // objet timer
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_; // objet publisher
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    // Initialise ROS 2 pour l'executable
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<MinimalPublisher>();
    // Créer le node et se met en attente de messages ou d'évènements du timer
    // Attention, cette fonction est bloquante !
    rclcpp::spin(node);
    // Coupe ROS 2 pour l'executable
    rclcpp::shutdown();
    return 0;
}
