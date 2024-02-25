#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class FakeDriverNode : public rclcpp::Node
{
public:
    FakeDriverNode() : Node("fake_driver_node")
    {
        // Initialize parameters
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("base_frame", "base_footprint");
        this->get_parameter("odom_frame", odom_frame_);
        this->get_parameter("base_frame", base_frame_);

        // Create subscribers and publishers
        vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 1000, std::bind(&FakeDriverNode::receiveCmd, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 50);
        odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Initialize odometry variables
        x_ = 0.0;
        y_ = 0.0;
        th_ = 0.0;

        current_time_ = this->now();
        last_time_ = this->now();

        // Set up a timer to publish odometry at a regular rate
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
            std::bind(&FakeDriverNode::publishOdometry, this));
    }

private:
    void receiveCmd(const geometry_msgs::msg::Twist::SharedPtr input_velocity)
    {
        receive_time_ = this->now();
        vx_ = input_velocity->linear.x;
        vy_ = input_velocity->linear.y;
        vth_ = input_velocity->angular.z;
    }

    void publishOdometry()
{
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    if (dt > 0.5)
    {
        vx_ = 0.0;
        vy_ = 0.0;
        vth_ = 0.0;
    }


    // Compute odometry as before
    double delta_x = (vx_ * std::cos(th_) - vy_ * std::sin(th_)) * dt;
    double delta_y = (vx_ * std::sin(th_) + vy_ * std::cos(th_)) * dt;
    double delta_th = vth_ * dt;

    x_ += delta_x;
    y_ += delta_y;
    th_ += delta_th;

    // Publish odometry message
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = odom_frame_;

    // Set the position
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0, 0, th_);
    tf2::convert(odom_quat, odom.pose.pose.orientation);

    // Set the velocity
    odom.child_frame_id = base_frame_;
    odom.twist.twist.linear.x = vx_;
    odom.twist.twist.linear.y = vy_;
    odom.twist.twist.angular.z = vth_;

    odom_pub_->publish(odom);

    // Publish the transform
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = odom_frame_;
    odom_trans.child_frame_id = base_frame_;

    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf2::toMsg(odom_quat);

    odom_broadcaster_->sendTransform(odom_trans);

    //last_time_ = current_time;
    last_time_ = this->now();
}

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string odom_frame_;
    std::string base_frame_;

    double x_;
    double y_;
    double th_;

    double vx_;
    double vy_;
    double vth_;

    rclcpp::Time receive_time_;
    rclcpp::Time current_time_;
    rclcpp::Time last_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeDriverNode>());
    rclcpp::shutdown();
    return 0;
}