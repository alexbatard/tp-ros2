#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "example_interfaces/srv/add_two_strings.hpp"
// include each message type you want to use

using namespace std::chrono_literals;

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
    rclcpp::TimerBase::SharedPtr timer_;                             // objet timer
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_; // objet publisher
};

void add(const std::shared_ptr<example_interfaces::srv::AddTwoStrings::Request> request,
         std::shared_ptr<example_interfaces::srv::AddTwoStrings::Response> response)
{
    // Concatenate the two strings and assign the result to the response
    response->sum = request->a + request->b;

    // Log the incoming request and the outgoing response
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: '%s', b: '%s'",
                request->a.c_str(), request->b.c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back response: '%s'", response->sum.c_str());
}

int main(int argc, char **argv)
{
    // Initialise ROS 2 pour l'executable
    rclcpp::init(argc, argv);

    // Créer un node générique, vous pouvez utiliser celui déjà crée
    std::shared_ptr<rclcpp::Node> node = rclcpp::std::make_shared<MinimalPublisher>();
    // Créer un service dans le node
    rclcpp::Service<example_interfaces::srv::AddTwoStrings>::SharedPtr service =
        node->create_service<example_interfaces::srv::AddTwoStrings>("add_two_strings", &add);
    // Met en attente le node en écoutant d'éventuelles demandes de service
    rclcpp::spin(node);

    // Créer le node et se met en attente de messages ou d'évènements du timer
    // Attention, cette fonction est bloquante !
    // rclcpp::spin(std::make_shared<MinimalPublisher>());

    // Coupe ROS 2 pour l'executable
    rclcpp::shutdown();
    return 0;
}
