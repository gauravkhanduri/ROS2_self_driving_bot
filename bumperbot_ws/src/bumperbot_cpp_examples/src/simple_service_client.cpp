#include <rclcpp/rclcpp.hpp>
#include <bumperbot_msgs/srv/add_two_ints.hpp>


class SimpleServiceClient : public rclcpp::Node
{
    public:
        SimpleServiceClient(int a, int b) : Node("simple_service_client")
        {
            client_ = create_client<bumperbot_msgs::srv::AddTwoInts>("add_two_ints");
           
            auto request = std::make_shared<bumperbot_msgs::srv::AddTwoInts::Request>();
            request->a = a;
            request->b = b;
            while (!client_->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(get_logger(), "service not available, waiting again...");
            }
            auto result = client_->async_send_request(request, std::bind(&SimpleServiceClient::handleResponse, this, std::placeholders::_1));
           

        } 
    private:
        rclcpp::Client<bumperbot_msgs::srv::AddTwoInts>::SharedPtr client_;
        void handleResponse(rclcpp::Client<bumperbot_msgs::srv::AddTwoInts>::SharedFuture future)
        {
            auto response = future.get();
            RCLCPP_INFO(get_logger(), "Response: %ld", response->sum);
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    if(argc != 3) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: simple_service_client a b");
        return 1;
    }
    
    int a = std::atoi(argv[1]);
    int b = std::atoi(argv[2]);
   
    rclcpp::spin(std::make_shared<SimpleServiceClient>(a,b));
    rclcpp::shutdown();
    return 0;
}