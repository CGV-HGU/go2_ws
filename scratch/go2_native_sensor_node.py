#!/usr/bin/env python3
"""
========================================================================================
🏆 Unitree Go2 Master Native Sensor & Timestamp Synchronizer Node
========================================================================================
Subscribes to Unitree Go2 Native CycloneDDS:
1. "lowstate" / "rt/lowstate" / "lf/lowstate" (unitree_go/msg/LowState)
   -> Publishes: /imu (sensor_msgs/Imu @ 50Hz stamped with Jetson now())
   -> Publishes: /joint_states (sensor_msgs/JointState @ 50Hz)
2. "sportmodestate" / "lf/sportmodestate" (unitree_go/msg/SportModeState)
   -> Publishes: /odom (nav_msgs/Odometry @ 50Hz)
   -> Publishes: Continuous 50Hz TF (odom -> base_link)
3. "/utlidar/cloud" / "/utlidar/cloud_deskewed" (sensor_msgs/msg/PointCloud2)
   -> Publishes: /pointcloud (15.7Hz stamped with Jetson now(), frame_id="radar")
   -> Completely eliminates the 228-second MCU-to-Jetson clock offset!
4. /cmd_vel (geometry_msgs/Twist)
   -> Publishes: /api/sport/request (unitree_api/msg/Request API 1008)
========================================================================================
"""

import json
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# Standard ROS 2 Messages
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState, PointCloud2
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster

# Unitree Official Messages
from unitree_go.msg import LowState, SportModeState
try:
    from unitree_api.msg import Request
    HAS_UNITREE_API = True
except ImportError:
    HAS_UNITREE_API = False


class Go2NativeSensorNode(Node):
    def __init__(self):
        super().__init__('go2_native_sensor_node')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Internal pose state for continuous 50Hz TF broadcasting
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.quat_x = 0.0
        self.quat_y = 0.0
        self.quat_z = 0.0
        self.quat_w = 1.0
        
        # QoS profiles
        sensor_sub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # 1. Standard ROS 2 Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu', pub_qos)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', pub_qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', pub_qos)
        self.lidar_pub = self.create_publisher(PointCloud2, '/pointcloud', pub_qos)
        
        if HAS_UNITREE_API:
            self.sport_req_pub = self.create_publisher(Request, '/api/sport/request', 10)
        else:
            self.sport_req_pub = None
        
        # 2. Subscribe to LowState (IMU + 12 Motors)
        self.lowstate_sub = self.create_subscription(LowState, 'lowstate', self.lowstate_callback, sensor_sub_qos)
        self.rt_lowstate_sub = self.create_subscription(LowState, 'rt/lowstate', self.lowstate_callback, sensor_sub_qos)
        self.lf_lowstate_sub = self.create_subscription(LowState, 'lf/lowstate', self.lowstate_callback, sensor_sub_qos)
        
        # 3. Subscribe to SportModeState (High-level Odom)
        self.sport_sub = self.create_subscription(SportModeState, 'sportmodestate', self.sport_callback, sensor_sub_qos)
        self.lf_sport_sub = self.create_subscription(SportModeState, 'lf/sportmodestate', self.sport_callback, sensor_sub_qos)
        
        # 4. Subscribe to Single Native 4D LiDAR (Motion-Deskewed) and Synchronize to /pointcloud
        self.create_subscription(PointCloud2, '/utlidar/cloud_deskewed', self.lidar_callback, sensor_sub_qos)
        
        # 5. Subscribe to /cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # 6. Continuous 50Hz TF Broadcaster Timer
        self.tf_timer = self.create_timer(1.0 / 50.0, self.tf_timer_callback)
        
        self.get_logger().info('🚀 [SENSOR NODE] Master Go2 Sensor & Timestamp Synchronizer Active (50Hz TF + 15.7Hz /pointcloud)!')

    def lidar_callback(self, msg: PointCloud2):
        # Restamp with Jetson system clock to ensure 0ms offset with camera and SLAM
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'radar'
        self.lidar_pub.publish(msg)

    def lowstate_callback(self, msg: LowState):
        now = self.get_clock().now().to_msg()
        
        # 1. IMU Message
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.orientation.w = float(msg.imu_state.quaternion[0])
        imu_msg.orientation.x = float(msg.imu_state.quaternion[1])
        imu_msg.orientation.y = float(msg.imu_state.quaternion[2])
        imu_msg.orientation.z = float(msg.imu_state.quaternion[3])
        imu_msg.angular_velocity.x = float(msg.imu_state.gyroscope[0])
        imu_msg.angular_velocity.y = float(msg.imu_state.gyroscope[1])
        imu_msg.angular_velocity.z = float(msg.imu_state.gyroscope[2])
        imu_msg.linear_acceleration.x = float(msg.imu_state.accelerometer[0])
        imu_msg.linear_acceleration.y = float(msg.imu_state.accelerometer[1])
        imu_msg.linear_acceleration.z = float(msg.imu_state.accelerometer[2])
        self.imu_pub.publish(imu_msg)
        
        # 2. JointState Message
        joint_msg = JointState()
        joint_msg.header.stamp = now
        joint_names = [
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint'
        ]
        joint_msg.name = joint_names
        joint_msg.position = [float(msg.motor_state[i].q) for i in range(12)]
        joint_msg.velocity = [float(msg.motor_state[i].dq) for i in range(12)]
        joint_msg.effort = [float(msg.motor_state[i].tau_est) for i in range(12)]
        self.joint_pub.publish(joint_msg)

    def sport_callback(self, msg: SportModeState):
        now = self.get_clock().now().to_msg()
        
        # Update internal pose state
        self.pos_x = float(msg.position[0])
        self.pos_y = float(msg.position[1])
        self.pos_z = float(msg.position[2])
        self.quat_w = float(msg.imu_state.quaternion[0])
        self.quat_x = float(msg.imu_state.quaternion[1])
        self.quat_y = float(msg.imu_state.quaternion[2])
        self.quat_z = float(msg.imu_state.quaternion[3])
        
        # Publish Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.pos_x
        odom_msg.pose.pose.position.y = self.pos_y
        odom_msg.pose.pose.position.z = self.pos_z
        odom_msg.pose.pose.orientation.w = self.quat_w
        odom_msg.pose.pose.orientation.x = self.quat_x
        odom_msg.pose.pose.orientation.y = self.quat_y
        odom_msg.pose.pose.orientation.z = self.quat_z
        odom_msg.twist.twist.linear.x = float(msg.velocity[0])
        odom_msg.twist.twist.linear.y = float(msg.velocity[1])
        odom_msg.twist.twist.linear.z = float(msg.velocity[2])
        odom_msg.twist.twist.angular.z = float(msg.yaw_speed)
        self.odom_pub.publish(odom_msg)

    def tf_timer_callback(self):
        now = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.pos_x
        t.transform.translation.y = self.pos_y
        t.transform.translation.z = self.pos_z
        t.transform.rotation.w = self.quat_w
        t.transform.rotation.x = self.quat_x
        t.transform.rotation.y = self.quat_y
        t.transform.rotation.z = self.quat_z
        self.tf_broadcaster.sendTransform(t)

    def cmd_vel_callback(self, msg: Twist):
        if self.sport_req_pub is not None:
            req = Request()
            req.header.identity.api_id = 1008
            param_dict = {"x": float(msg.linear.x), "y": float(msg.linear.y), "z": float(msg.angular.z)}
            req.parameter = json.dumps(param_dict)
            self.sport_req_pub.publish(req)


def main():
    rclpy.init()
    node = Go2NativeSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
