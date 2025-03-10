#include "bumperbot_utils/bumperbot_trajectory.hpp"

BumperbotTrajectory::BumperbotTrajectory(const std::string& name) : Node(name)
{
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/bumperbot_controller/trajectory", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/bumperbot_controller/odom", 10, 
                std::bind(&BumperbotTrajectory::odomCallback, this, std::placeholders::_1));
}

void BumperbotTrajectory::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "Received odometry message: x = %f, y = %f, theta = %f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.orientation.z);
    
    trajectory_.header.frame_id = msg->header.frame_id;
    geometry_msgs::msg::PoseStamped curr_pose;
    curr_pose.header.frame_id = msg->header.frame_id;
    curr_pose.header.stamp = msg->header.stamp;
    curr_pose.pose = msg->pose.pose;
    trajectory_.poses.push_back(curr_pose);
    path_pub_->publish(trajectory_);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BumperbotTrajectory>("bumperbot_trajectory"));
    rclcpp::shutdown();
    return 0;
}