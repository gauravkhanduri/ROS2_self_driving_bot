#ifndef BUMPERBOT_TRAJECTORY_HPP_
#define BUMPERBOT_TRAJECTORY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>



class BumperbotTrajectory : public rclcpp::Node
{
    public:
        BumperbotTrajectory(const std::string& name);

    private:
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;



        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

        nav_msgs::msg::Path trajectory_;


};

#endif // BUMPERBOT_TRAJECTORY_HPP_