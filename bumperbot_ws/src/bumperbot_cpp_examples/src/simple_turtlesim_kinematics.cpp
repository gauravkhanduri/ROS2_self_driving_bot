#include<bumperbot_cpp_examples/simple_turtlesim_kinematics.hpp>

// Constructor
SimpleTurtlesimKinematics::SimpleTurtlesimKinematics(const std::string& name): Node(name)
{
    // Create a subscription to the turtle1 pose topic
    turtle1_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
        "/turtle1/pose", 10, std::bind(&SimpleTurtlesimKinematics::turtle1PoseCallback, this, std::placeholders::_1));

    // Create a subscription to the turtle2 pose topic
    turtle2_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
        "/turtle2/pose", 10, std::bind(&SimpleTurtlesimKinematics::turtle2PoseCallback, this, std::placeholders::_1));
}

// Callback function for the turtle1 pose subscription
void SimpleTurtlesimKinematics::turtle1PoseCallback(const turtlesim::msg::Pose::SharedPtr pose)
{
    // Store the last turtle1 pose
    last_turtle1_pose_ = *pose;
}


// Callback function for the turtle2 pose subscription
void SimpleTurtlesimKinematics::turtle2PoseCallback(const turtlesim::msg::Pose::SharedPtr pose)
{
    // Store the last turtle2 pose
    last_turtle2_pose_ = *pose;
    float Tx = last_turtle2_pose_.x - last_turtle1_pose_.x;
    float Ty = last_turtle2_pose_.y - last_turtle1_pose_.y;

    float theta_redian = last_turtle2_pose_.theta - last_turtle1_pose_.theta;
    float theta_degree = theta_redian * 180 / 3.14;

    RCLCPP_INFO_STREAM(this->get_logger(), "Tx: " << Tx << " Ty: " << Ty <<"\n"<< "Radian" << theta_redian<< "\n"<<" Theta: " << theta_degree << "\n"
                                        "|R11        R12|:   |" << std::cos(theta_redian)<<"\t"<< -std::sin(theta_redian)<<"|\n"<<
                                        "|R21        R22|:   |" << std::sin(theta_redian)<<"\t"<< std::cos(theta_redian)<<"|\n");


    //
}

int main(int argc, char** argv)
{
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create an instance of the SimpleTurtlesimKinematics class
    auto node = std::make_shared<SimpleTurtlesimKinematics>("simple_turtlesim_kinematics");

    // Spin the node so the callback functions can be called
    rclcpp::spin(node);

    // Shut down the ROS 2 system
    rclcpp::shutdown();

    return 0;
}