#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>

using namespace std::chrono_literals;

class SimplePublisher : public rclcpp::Node
{
public:
    SimplePublisher() : Node("simple_publisher"), count_(0)
    {
        publisher_ = create_publisher<std_msgs::msg::String>("chatter", 10);
        timer_ = create_wall_timer(1s, std::bind(&SimplePublisher::timerCallback, this));
    }

private:
    unsigned int count_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timerCallback()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello ROS2 - counter" + std::to_string(count_++);

        publisher_->publish(msg);

    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<SimplePublisher>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0 ;

}
