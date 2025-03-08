#ifndef SIMPLE_TURTLESIM_KINEMATICS_HPP_
#define SIMPLE_TURTLESIM_KINEMATICS_HPP_

#include<rclcpp/rclcpp.hpp>
#include<turtlesim/msg/pose.hpp>

// This class inherits from rclcpp::Node
class SimpleTurtlesimKinematics : public rclcpp::Node
{
    public:
        // Constructor
        SimpleTurtlesimKinematics(const std::string& name);

    private:
        void turtle1PoseCallback(const turtlesim::msg::Pose::SharedPtr pose);
        void turtle2PoseCallback(const turtlesim::msg::Pose::SharedPtr pose);

        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle1_pose_sub_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle2_pose_sub_;

        turtlesim::msg::Pose last_turtle1_pose_;
        turtlesim::msg::Pose last_turtle2_pose_;


};

#endif  // SIMPLE_TURTLESIM_KINEMATICS_HPP_