#!/usr/bin/env python3
"""
========================================================================================
🏆 Unitree Go2 Full Native Sensor Bringup Node (IMU + Odom + Joint States + CmdVel)
========================================================================================
Subscribes to Unitree Go2 Native CycloneDDS:
1. "lowstate" (unitree_go/msg/LowState - Best Effort)
   -> Publishes: /imu (sensor_msgs/Imu @ 50Hz) - Quaternion, Gyro, Accel
   -> Publishes: /joint_states (sensor_msgs/JointState @ 50Hz) - 12 Motor Angles
2. "sportmodestate" / "lf/sportmodestate" (unitree_go/msg/SportModeState - Best Effort)
   -> Publishes: /odom (nav_msgs/Odometry @ 50Hz) + High-rate TF (odom -> base_link @ 50Hz)
3. /cmd_vel (geometry_msgs/Twist)
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
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster

# Unitree Official Messages
from unitree_go.msg import LowState, SportModeState
from unitree_api.msg import Request

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
        self.sport_req_pub = self.create_publisher(Request, '/api/sport/request', 10)
        
        # 2. Subscribe to LowState (IMU + 12 Motors)
        self.lowstate_sub = self.create_subscription(
            LowState,
            'lowstate',
            self.lowstate_callback,
            sensor_sub_qos
        )
        self.rt_lowstate_sub = self.create_subscription(
            LowState,
            'rt/lowstate',
            self.lowstate_callback,
            sensor_sub_qos
        )
        self.lf_lowstate_sub = self.create_subscription(
            LowState,
            'lf/lowstate',
            self.lowstate_callback,
            sensor_sub_qos
        )
        
        # 3. Subscribe to SportModeState (High-level Odom)
        self.sport_sub = self.create_subscription(
            SportModeState,
            'sportmodestate',
            self.sport_callback,
            sensor_sub_qos
        )
        self.lf_sport_sub = self.create_subscription(
            SportModeState,
            'lf/sportmodestate',
            self.sport_callback,
            sensor_sub_qos
        )
        self.rt_sport_sub = self.create_subscription(
            SportModeState,
            'rt/sportmodestate',
            self.sport_callback,
            sensor_sub_qos
        )
        
        # 4. Subscribe to /cmd_vel
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.joint_names = [
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"
        ]

        # 5. Continuous 50Hz TF Timer (odom -> base_link)
        self.tf_timer = self.create_timer(1.0 / 50.0, self.tf_timer_callback)

        self.get_logger().info("🚀 [SENSOR NODE] Go2 Native Sensor Suite Active (Best-Effort 50Hz TF Broadcaster)!")

    def tf_timer_callback(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self.pos_x
        tf.transform.translation.y = self.pos_y
        tf.transform.translation.z = self.pos_z
        tf.transform.rotation.x = self.quat_x
        tf.transform.rotation.y = self.quat_y
        tf.transform.rotation.z = self.quat_z
        tf.transform.rotation.w = self.quat_w
        self.tf_broadcaster.sendTransform(tf)

    # --------------------------------------------------------------------------
    # 1. LowState -> /imu & /joint_states
    # --------------------------------------------------------------------------
    def lowstate_callback(self, msg: LowState):
        now = self.get_clock().now().to_msg()
        
        # A. Publish Standard /imu
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = 'imu_link'
        
        # Go2 IMU Quaternion: [qw, qx, qy, qz]
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
        
        # B. Publish Standard /joint_states
        js_msg = JointState()
        js_msg.header.stamp = now
        js_msg.name = self.joint_names
        
        # Motor map: FL(3,4,5), FR(0,1,2), RL(9,10,11), RR(6,7,8)
        js_msg.position = [
            float(msg.motor_state[3].q), float(msg.motor_state[4].q), float(msg.motor_state[5].q),
            float(msg.motor_state[0].q), float(msg.motor_state[1].q), float(msg.motor_state[2].q),
            float(msg.motor_state[9].q), float(msg.motor_state[10].q), float(msg.motor_state[11].q),
            float(msg.motor_state[6].q), float(msg.motor_state[7].q), float(msg.motor_state[8].q)
        ]
        self.joint_pub.publish(js_msg)

    # --------------------------------------------------------------------------
    # 2. SportModeState -> /odom & TF update
    # --------------------------------------------------------------------------
    def sport_callback(self, msg: SportModeState):
        now = self.get_clock().now().to_msg()
        
        self.pos_x = float(msg.position[0])
        self.pos_y = float(msg.position[1])
        self.pos_z = float(msg.position[2]) + 0.07
        
        # Euler RPY to Quaternion
        roll, pitch, yaw = msg.imu_state.rpy[0], msg.imu_state.rpy[1], msg.imu_state.rpy[2]
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        
        self.quat_w = float(cr * cp * cy + sr * sp * sy)
        self.quat_x = float(sr * cp * cy - cr * sp * sy)
        self.quat_y = float(cr * sp * cy + sr * cp * sy)
        self.quat_z = float(cr * cp * sy - sr * sp * cy)
        
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.pos_x
        odom.pose.pose.position.y = self.pos_y
        odom.pose.pose.position.z = self.pos_z
        
        odom.pose.pose.orientation.w = self.quat_w
        odom.pose.pose.orientation.x = self.quat_x
        odom.pose.pose.orientation.y = self.quat_y
        odom.pose.pose.orientation.z = self.quat_z
        
        odom.twist.twist.linear.x = float(msg.velocity[0])
        odom.twist.twist.linear.y = float(msg.velocity[1])
        odom.twist.twist.linear.z = float(msg.velocity[2])
        odom.twist.twist.angular.z = float(msg.yaw_speed)
        
        self.odom_pub.publish(odom)

    # --------------------------------------------------------------------------
    # 3. /cmd_vel -> Official Move API
    # --------------------------------------------------------------------------
    def cmd_callback(self, msg: Twist):
        req = Request()
        req.header.identity.api_id = 1008
        param = {"x": float(msg.linear.x), "y": float(msg.linear.y), "z": float(msg.angular.z)}
        req.parameter = json.dumps(param)
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
