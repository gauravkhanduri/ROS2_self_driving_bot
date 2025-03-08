#include<bumperbot_cpp_examples/simple_tf_kinematics.hpp>

SimpleTfKinematics::SimpleTfKinematics(const std::string &name) : Node(name), x_increment_(0.05), rotations_counter_(0)
                                        , last_x_(0.0)
{
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    dynamic_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);



    static_transform_stamped_.header.stamp = get_clock()->now();
    static_transform_stamped_.header.frame_id = "bumperbot_base";
    static_transform_stamped_.child_frame_id = "bumperbot_top";
    static_transform_stamped_.transform.translation.x = 0.0;
    static_transform_stamped_.transform.translation.y = 0.0;
    static_transform_stamped_.transform.translation.z = 0.3;
    static_transform_stamped_.transform.rotation.x = 0.0;
    static_transform_stamped_.transform.rotation.y = 0.0;
    static_transform_stamped_.transform.rotation.z = 0.0;
    static_transform_stamped_.transform.rotation.w = 1.0;

    static_tf_broadcaster_->sendTransform(static_transform_stamped_);

    RCLCPP_INFO(get_logger(), "Published static transform from %s to %s", 
                static_transform_stamped_.header.frame_id.c_str(), 
                static_transform_stamped_.child_frame_id.c_str());

    timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&SimpleTfKinematics::timer_callback, this));

    get_transform_srv_ = create_service<bumperbot_msgs::srv::GetTransform>("get_transform", 
        std::bind(&SimpleTfKinematics::getTransformCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    last_orientation_.setRPY(0.0, 0.0, 0.0);
    orientation_increment_.setRPY(0.0, 0.0, 0.05);

}
void SimpleTfKinematics::timer_callback()
{
    dynamic_transform_stamped_.header.stamp = get_clock()->now();
    dynamic_transform_stamped_.header.frame_id = "odom";
    dynamic_transform_stamped_.child_frame_id = "bumperbot_base";
    dynamic_transform_stamped_.transform.translation.x = last_x_ + x_increment_;
    dynamic_transform_stamped_.transform.translation.y = 0.0;
    tf2::Quaternion orientation = last_orientation_ * orientation_increment_;
    orientation.normalize();
    dynamic_transform_stamped_.transform.translation.z = 0.0;
    dynamic_transform_stamped_.transform.rotation.x = orientation.x();
    dynamic_transform_stamped_.transform.rotation.y = orientation.y();
    dynamic_transform_stamped_.transform.rotation.z = orientation.z();
    dynamic_transform_stamped_.transform.rotation.w = orientation.w();

    dynamic_tf_broadcaster_->sendTransform(dynamic_transform_stamped_);

    last_x_ = dynamic_transform_stamped_.transform.translation.x;
    rotations_counter_++;
    last_orientation_ = orientation;

    if(rotations_counter_ == 10)
    {
       orientation_increment_ = orientation_increment_.inverse();
         rotations_counter_ = 0;
    }
}
bool SimpleTfKinematics::getTransformCallback(
    const std::shared_ptr<bumperbot_msgs::srv::GetTransform::Request> request,
    const std::shared_ptr<bumperbot_msgs::srv::GetTransform::Response> response)
{
    RCLCPP_INFO_STREAM(get_logger(), "Request to get transform from " << request->frame_id << " to " << request->child_frame_id);

    geometry_msgs::msg::TransformStamped transform_stamped;
    try
    {
        transform_stamped = tf_buffer_->lookupTransform(request->child_frame_id, request->frame_id, tf2::TimePointZero);
        response->transform = transform_stamped;
        response->success = true;
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR(get_logger(), "Transform error: %s", ex.what());
        response->success = false;
    }

    return true;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleTfKinematics>("simple_tf_kinematics"));
    rclcpp::shutdown();
    return 0;
}