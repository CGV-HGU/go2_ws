#include <iostream>
#include <cmath>
#include <memory>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/go2/LowState_.hpp>

class Go2SDK2SensorBridge : public rclcpp::Node
{
public:
    Go2SDK2SensorBridge() : Node("go2_sdk2_sensor_bridge")
    {
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        joint_names_ = {
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"
        };

        // Initialize Unitree SDK2 Channel on eth0
        unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");

        // Subscribe to Native Unitree DDS topics
        sport_sub_.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>("rt/sportmodestate"));
        sport_sub_->InitChannel(std::bind(&Go2SDK2SensorBridge::SportModeStateHandler, this, std::placeholders::_1), 1);

        low_sub_.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>("rt/lowstate"));
        low_sub_->InitChannel(std::bind(&Go2SDK2SensorBridge::LowStateHandler, this, std::placeholders::_1), 1);

        RCLCPP_INFO(this->get_logger(), "🚀 [SDK2 SENSOR BRIDGE] Connected to Go2 eth0 via Unitree SDK2 Native DDS!");
    }

private:
    void SportModeStateHandler(const void *message)
    {
        const auto *msg = (const unitree_go::msg::dds_::SportModeState_ *)message;
        auto now_time = this->get_clock()->now();

        double px = msg->position()[0];
        double py = msg->position()[1];
        double pz = msg->position()[2] + 0.07;

        double roll = msg->imu_state().rpy()[0];
        double pitch = msg->imu_state().rpy()[1];
        double yaw = msg->imu_state().rpy()[2];

        double cy = std::cos(yaw * 0.5);
        double sy = std::sin(yaw * 0.5);
        double cp = std::cos(pitch * 0.5);
        double sp = std::sin(pitch * 0.5);
        double cr = std::cos(roll * 0.5);
        double sr = std::sin(roll * 0.5);

        double qw = cr * cp * cy + sr * sp * sy;
        double qx = sr * cp * cy - cr * sp * sy;
        double qy = cr * sp * cy + sr * cp * sy;
        double qz = cr * cp * sy - sr * sp * cy;

        // 1. Broadcast TF (odom -> base_link)
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now_time;
        tf_msg.header.frame_id = "odom";
        tf_msg.child_frame_id = "base_link";
        tf_msg.transform.translation.x = px;
        tf_msg.transform.translation.y = py;
        tf_msg.transform.translation.z = pz;
        tf_msg.transform.rotation.x = qx;
        tf_msg.transform.rotation.y = qy;
        tf_msg.transform.rotation.z = qz;
        tf_msg.transform.rotation.w = qw;
        tf_broadcaster_->sendTransform(tf_msg);

        // 2. Publish Standard /odom
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position.x = px;
        odom_msg.pose.pose.position.y = py;
        odom_msg.pose.pose.position.z = pz;
        odom_msg.pose.pose.orientation.x = qx;
        odom_msg.pose.pose.orientation.y = qy;
        odom_msg.pose.pose.orientation.z = qz;
        odom_msg.pose.pose.orientation.w = qw;

        odom_msg.twist.twist.linear.x = msg->velocity()[0];
        odom_msg.twist.twist.linear.y = msg->velocity()[1];
        odom_msg.twist.twist.linear.z = msg->velocity()[2];
        odom_msg.twist.twist.angular.z = msg->yaw_speed();

        odom_pub_->publish(odom_msg);
    }

    void LowStateHandler(const void *message)
    {
        const auto *msg = (const unitree_go::msg::dds_::LowState_ *)message;
        auto now_time = this->get_clock()->now();

        // 1. Publish /imu
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = now_time;
        imu_msg.header.frame_id = "imu_link";
        imu_msg.orientation.w = msg->imu_state().quaternion()[0];
        imu_msg.orientation.x = msg->imu_state().quaternion()[1];
        imu_msg.orientation.y = msg->imu_state().quaternion()[2];
        imu_msg.orientation.z = msg->imu_state().quaternion()[3];

        imu_msg.angular_velocity.x = msg->imu_state().gyroscope()[0];
        imu_msg.angular_velocity.y = msg->imu_state().gyroscope()[1];
        imu_msg.angular_velocity.z = msg->imu_state().gyroscope()[2];

        imu_msg.linear_acceleration.x = msg->imu_state().accelerometer()[0];
        imu_msg.linear_acceleration.y = msg->imu_state().accelerometer()[1];
        imu_msg.linear_acceleration.z = msg->imu_state().accelerometer()[2];
        imu_pub_->publish(imu_msg);

        // 2. Publish /joint_states
        sensor_msgs::msg::JointState js_msg;
        js_msg.header.stamp = now_time;
        js_msg.name = joint_names_;
        js_msg.position = {
            msg->motor_state()[3].q(), msg->motor_state()[4].q(), msg->motor_state()[5].q(),
            msg->motor_state()[0].q(), msg->motor_state()[1].q(), msg->motor_state()[2].q(),
            msg->motor_state()[9].q(), msg->motor_state()[10].q(), msg->motor_state()[11].q(),
            msg->motor_state()[6].q(), msg->motor_state()[7].q(), msg->motor_state()[8].q()
        };
        joint_pub_->publish(js_msg);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::vector<std::string> joint_names_;

    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> sport_sub_;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> low_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Go2SDK2SensorBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
