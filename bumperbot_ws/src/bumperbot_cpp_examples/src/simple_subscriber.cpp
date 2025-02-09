#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>


// The std::bind function is used to bind the member function msgCallback to the class SimpleSubscriber.
// The _1 is a placeholder for the argument of the callback function.
// The std::placeholders namespace is used to define the placeholder _1.
// The placeholder _1 is used to bind the argument of the callback function to the argument of the member function.



using std::placeholders::_1;

class SimpleSubscriber : public rclcpp::Node
{
public:
    SimpleSubscriber() : Node("simple_subscriber")
    {
        sub_ = this->create_subscription<std_msgs::msg::String>("chatter", 10, std::bind(&SimpleSubscriber::msgCallback, this, _1));
        RCLCPP_INFO(get_logger(),"I am here");
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

    void msgCallback(const std_msgs::msg::String &msg)
    {
        RCLCPP_INFO_STREAM(get_logger(), "I heard " << msg.data.c_str());
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<SimpleSubscriber>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}