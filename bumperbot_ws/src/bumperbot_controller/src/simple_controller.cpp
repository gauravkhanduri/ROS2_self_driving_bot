#include <bumperbot_controller/simple_controller.hpp>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>

SimpleController::SimpleController(const std::string& name) : Node(name), left_wheel_prev_pos_(0.0), right_wheel_prev_pos_(0.0)
    , x_(0.0)
    , y_(0.0)
    , theta_(0.0)
{
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("wheel_separation", 0.17);

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    wheel_separation_ = get_parameter("wheel_separation").as_double();

    RCLCPP_INFO_STREAM(get_logger(), "wheel_radius: " << wheel_radius_);
    RCLCPP_INFO_STREAM(get_logger(), "wheel_separation: " << wheel_separation_);

    prev_time_ = get_clock()->now();

    wheel_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/simple_velocity_controller/commands", 10);
    vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("/bumperbot_controller/cmd_vel", 10, 
        std::bind(&SimpleController::velCallback, this, std::placeholders::_1));
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, 
                std::bind(&SimpleController::jointCallback, this, std::placeholders::_1));
    
    // odometry publisher
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/bumperbot_controller/odom", 10);

    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_footprint";
    odom_msg_.pose.pose.position.x = 0.0;
    odom_msg_.pose.pose.position.y = 0.0;
    odom_msg_.pose.pose.position.z = 0.0;
    odom_msg_.pose.pose.orientation.w = 1.0;
    
    speed_conversion_ << wheel_radius_/2, wheel_radius_/2,
                        wheel_radius_/wheel_separation_, -wheel_radius_/wheel_separation_;
    
    RCLCPP_INFO_STREAM(get_logger(), "The conversion matrix is: " << speed_conversion_);
}

void SimpleController::velCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "Received velocity command: linear.x = %f, angular.z = %f", msg->twist.linear.x, msg->twist.angular.z);  // Added logging
    //print I am in
    RCLCPP_INFO(get_logger(), "I am in velCallback");  // Added logging
    

    Eigen::Vector2d robot_speed(msg->twist.linear.x, msg->twist.angular.z);
    Eigen::Vector2d wheel_speed = speed_conversion_.inverse() * robot_speed;

    std_msgs::msg::Float64MultiArray wheel_speed_msg;
    wheel_speed_msg.data.push_back(wheel_speed.coeff(1));
    wheel_speed_msg.data.push_back(wheel_speed.coeff(0));

    RCLCPP_INFO(get_logger(), "Publishing wheel speeds: left = %f, right = %f", wheel_speed.coeff(1), wheel_speed.coeff(0));  // Added logging

    wheel_cmd_pub_->publish(wheel_speed_msg);
}

void SimpleController::jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{


    double dp_left = msg->position.at(1) - left_wheel_prev_pos_;
    double dp_right = msg->position.at(0) - right_wheel_prev_pos_;

    rclcpp::Time msg_time = msg->header.stamp;
    rclcpp::Duration dt = msg_time - prev_time_;

    left_wheel_prev_pos_ = msg->position.at(1);
    right_wheel_prev_pos_ = msg->position.at(0);

    prev_time_ = msg_time;

    double fi_left = dp_left / dt.seconds();
    double fi_right = dp_right / dt.seconds();

    double linear = (wheel_radius_ * fi_right + wheel_radius_ * fi_left) / 2;
    double angular = (wheel_radius_ * fi_right - wheel_radius_ * fi_left) / wheel_separation_;

    double d_s = (wheel_radius_ * dp_right + wheel_radius_ * dp_left)/2;
    double d_theta = (wheel_radius_ * dp_right - wheel_radius_ * dp_left) / wheel_separation_;

    theta_ += d_theta;
    x_ += d_s * cos(theta_);
    y_ += d_s * sin(theta_);

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom_msg_.pose.pose.orientation.x = q.x();
    odom_msg_.pose.pose.orientation.y = q.y();
    odom_msg_.pose.pose.orientation.z = q.z();
    odom_msg_.pose.pose.orientation.w = q.w();

    odom_msg_.header.stamp = get_clock()->now();
    odom_msg_.pose.pose.position.x = x_;
    odom_msg_.pose.pose.position.y = y_;

    odom_msg_.twist.twist.linear.x = linear;
    odom_msg_.twist.twist.angular.z = angular;

    odom_pub_->publish(odom_msg_);


    


    RCLCPP_INFO_STREAM(get_logger(), "Robot pose: x = " << x_ << ", y = " << y_ << ", theta = " << theta_);

    RCLCPP_INFO_STREAM(get_logger(), "Linear velocity: " << linear << ", Angular velocity: " << angular);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleController>("simple_controller"));
    rclcpp::shutdown();
    return 0;
}