#include <rclcpp/rclcpp.hpp>
#include <bumperbot_msgs/srv/add_two_ints.hpp>


class SimpleServiceServer : public rclcpp::Node
{
public:
    SimpleServiceServer() : Node("simple_service_server")
    {
        service_ = create_service<bumperbot_msgs::srv::AddTwoInts>("add_two_ints", 
            std::bind(&SimpleServiceServer::serviceCallback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO_STREAM(get_logger(), "Simple Service Server has been started.");
    }
private:
    rclcpp::Service<bumperbot_msgs::srv::AddTwoInts>::SharedPtr service_;

    void serviceCallback(std::shared_ptr<bumperbot_msgs::srv::AddTwoInts::Request> request,
                         std::shared_ptr<bumperbot_msgs::srv::AddTwoInts::Response> response)
    {
        RCLCPP_INFO_STREAM(get_logger(), "Incoming request\na: " << request->a << "\nb: " << request->b);
        response->sum = request->a + request->b;
        RCLCPP_INFO(get_logger(), "Incoming request\na: %ld\nb: %ld", request->a, request->b);
        RCLCPP_INFO(get_logger(), "Sending response: %ld", response->sum);
    }

};
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleServiceServer>());
    rclcpp::shutdown();
    return 0;
}
