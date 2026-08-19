#!/usr/bin/env python3
"""
========================================================================================
🏆 Unitree Go2 Full Native Sensor Bringup Node (IMU + Odom + Camera + Joint States)
========================================================================================
Subscribes to Unitree Go2 Native DDS:
1. "lowstate" (unitree_go/msg/LowState)
   -> Publishes: /imu (sensor_msgs/Imu @ 10~50Hz) - Quaternion, Gyro, Accel
   -> Publishes: /joint_states (sensor_msgs/JointState @ 10~50Hz) - 12 Motor Angles
2. "sportmodestate" / "lf/sportmodestate" (unitree_go/msg/SportModeState)
   -> Publishes: /odom (nav_msgs/Odometry @ 50Hz) + TF (odom -> base_link)
3. GStreamer H.264 (230.1.1.1:1720)
   -> Publishes: /camera/front/image_raw (sensor_msgs/Image @ 30fps)
4. /cmd_vel (geometry_msgs/Twist)
   -> Publishes: /api/sport/request (unitree_api/msg/Request)
========================================================================================
"""

import json
import math
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

# Standard ROS 2 Messages
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image, JointState, PointCloud2
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge

# Unitree Official Messages
from unitree_go.msg import LowState, SportModeState
from unitree_api.msg import Request

class Go2NativeSensorNode(Node):
    def __init__(self):
        super().__init__('go2_native_sensor_node')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.bridge = CvBridge()
        
        # 1. Standard ROS 2 Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/pointcloud', 10)
        self.camera_pub = self.create_publisher(Image, '/camera/front/image_raw', 10)
        self.sport_req_pub = self.create_publisher(Request, '/api/sport/request', 10)
        
        # 2. Subscribe to LiDAR PointCloud
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            'utlidar/cloud',
            self.lidar_callback,
            10
        )
        self.rt_lidar_sub = self.create_subscription(
            PointCloud2,
            'rt/utlidar/cloud',
            self.lidar_callback,
            10
        )
        
        # 2. Subscribe to LowState (IMU + 12 Motors)
        self.lowstate_sub = self.create_subscription(
            LowState,
            'lowstate',
            self.lowstate_callback,
            10
        )
        
        # 3. Subscribe to SportModeState (High-level Odom)
        self.sport_sub = self.create_subscription(
            SportModeState,
            'sportmodestate',
            self.sport_callback,
            10
        )
        self.lf_sport_sub = self.create_subscription(
            SportModeState,
            'lf/sportmodestate',
            self.sport_callback,
            10
        )
        
        # 4. Subscribe to /cmd_vel
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )
        
        # 5. Front Camera GStreamer (30fps)
        pipeline = (
            'udpsrc address=230.1.1.1 port=1720 multicast-group=230.1.1.1 auto-multicast=true ! '
            'application/x-rtp, media=video, clock-rate=90000, payload=96, encoding-name=H264 ! '
            'rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink drop=true max-buffers=1'
        )
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if self.cap.isOpened():
            self.get_logger().info("✅ [CAMERA] Unitree Go2 Front Camera Connected! (30 fps)")
            self.cam_timer = self.create_timer(1.0 / 30.0, self.camera_callback)
        else:
            self.get_logger().warn("⚠️ [CAMERA] GStreamer not opened, retrying...")

        self.joint_names = [
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"
        ]
        self.get_logger().info("🚀 [SENSOR NODE] Unitree Go2 Native Sensor Suite Active!")

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
    # 2. SportModeState -> /odom & TF
    # --------------------------------------------------------------------------
    def sport_callback(self, msg: SportModeState):
        now = self.get_clock().now().to_msg()
        
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = float(msg.position[0])
        odom.pose.pose.position.y = float(msg.position[1])
        odom.pose.pose.position.z = float(msg.position[2]) + 0.07
        
        # Euler RPY to Quaternion
        roll, pitch, yaw = msg.imu_state.rpy[0], msg.imu_state.rpy[1], msg.imu_state.rpy[2]
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        
        odom.pose.pose.orientation.w = float(cr * cp * cy + sr * sp * sy)
        odom.pose.pose.orientation.x = float(sr * cp * cy - cr * sp * sy)
        odom.pose.pose.orientation.y = float(cr * sp * cy + sr * cp * sy)
        odom.pose.pose.orientation.z = float(cr * cp * sy - sr * sp * cy)
        
        odom.twist.twist.linear.x = float(msg.velocity[0])
        odom.twist.twist.linear.y = float(msg.velocity[1])
        odom.twist.twist.linear.z = float(msg.velocity[2])
        odom.twist.twist.angular.z = float(msg.yaw_speed)
        
        self.odom_pub.publish(odom)
        
        # Broadcast TF
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.translation.z = odom.pose.pose.position.z
        tf.transform.rotation = odom.pose.pose.orientation
    # --------------------------------------------------------------------------
    # 2-B. LiDAR Callback -> /pointcloud
    # --------------------------------------------------------------------------
    def lidar_callback(self, msg: PointCloud2):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'radar'
        self.pointcloud_pub.publish(msg)

    # --------------------------------------------------------------------------
    # 3. Camera Callback
    # --------------------------------------------------------------------------
    def camera_callback(self):
        if not self.cap.isOpened():
            return
        ret, frame = self.cap.read()
        if ret and frame is not None:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_link'
            self.camera_pub.publish(img_msg)

    # --------------------------------------------------------------------------
    # 4. /cmd_vel -> Official Move API
    # --------------------------------------------------------------------------
    def cmd_callback(self, msg: Twist):
        req = Request()
        req.header.identity.api_id = 1008
        param = {"x": float(msg.linear.x), "y": float(msg.linear.y), "z": float(msg.angular.z)}
        req.parameter = json.dumps(param)
        self.sport_req_pub.publish(req)

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

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
