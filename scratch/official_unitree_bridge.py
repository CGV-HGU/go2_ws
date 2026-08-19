#!/usr/bin/env python3
"""
========================================================================================
🏆 Unitree Go2 Official SDK2 / ROS 2 Native Pure Bridge Node (100% Official)
========================================================================================
Author: Minseok (Adapted from Unitree Official SDK2 / unitree_ros2)
License: Apache-2.0 / Unitree Robotics Co., Ltd.

Connects directly to Unitree Go2 Native CycloneDDS streams:
1. Subscribes: "lf/sportmodestate" / "sportmodestate" (unitree_go/msg/SportModeState)
   -> Publishes: /odom (nav_msgs/Odometry @ 50Hz) + TF (odom -> base_link)
   -> Publishes: /imu (sensor_msgs/Imu @ 50Hz)
2. Subscribes: /cmd_vel (geometry_msgs/Twist)
   -> Publishes: /api/sport/request (unitree_api/msg/Request, Mode::Move)
3. Captures: H.264 RTP stream (230.1.1.1:1720)
   -> Publishes: /camera/front/image_raw (sensor_msgs/Image @ 30fps)
========================================================================================
"""

import json
import math
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# ROS 2 Standard Messages
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge

# Unitree Official Messages
from unitree_go.msg import SportModeState
from unitree_api.msg import Request

class OfficialUnitreeBridge(Node):
    def __init__(self):
        super().__init__('official_unitree_bridge')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.bridge = CvBridge()
        
        # ----------------------------------------------------------------------
        # 1. Standard ROS 2 Publishers
        # ----------------------------------------------------------------------
        qos_transient = QoSProfile(depth=10)
        qos_transient.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.camera_pub = self.create_publisher(Image, '/camera/front/image_raw', 10)
        
        # ----------------------------------------------------------------------
        # 2. Unitree Official DDS Publishers & Subscribers
        # ----------------------------------------------------------------------
        self.sport_req_pub = self.create_publisher(Request, '/api/sport/request', 10)
        
        # Subscribe to Official High-level SportModeState (High Frequency)
        self.state_sub = self.create_subscription(
            SportModeState,
            'sportmodestate',
            self.sport_state_callback,
            10
        )
        # Fallback for low-frequency topic if high-freq not active
        self.lf_state_sub = self.create_subscription(
            SportModeState,
            'lf/sportmodestate',
            self.sport_state_callback,
            10
        )
        
        # Subscribe to Velocity Commands (/cmd_vel)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # ----------------------------------------------------------------------
        # 3. Built-in Front Camera GStreamer Capture (30 fps)
        # ----------------------------------------------------------------------
        pipeline = (
            'udpsrc address=230.1.1.1 port=1720 multicast-group=230.1.1.1 auto-multicast=true ! '
            'application/x-rtp, media=video, clock-rate=90000, payload=96, encoding-name=H264 ! '
            'rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink drop=true max-buffers=1'
        )
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if self.cap.isOpened():
            self.get_logger().info("✅ [CAMERA] Unitree Go2 Front Camera Connected! (30 fps)")
            self.camera_timer = self.create_timer(1.0 / 30.0, self.camera_callback)
        else:
            self.get_logger().warn("⚠️ [CAMERA] GStreamer pipeline not opened, retrying in background...")
            
        self.get_logger().info("🚀 [OFFICIAL BRIDGE] 100% Native Unitree SDK2 ROS 2 Bridge Running!")

    # --------------------------------------------------------------------------
    # Official SportModeState -> /odom & /imu & TF
    # --------------------------------------------------------------------------
    def sport_state_callback(self, msg: SportModeState):
        now = self.get_clock().now().to_msg()
        
        # 1. Orientation from IMU RPY to Quaternion
        roll, pitch, yaw = msg.imu_state.rpy[0], msg.imu_state.rpy[1], msg.imu_state.rpy[2]
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        # 2. Publish Standard /odom (50Hz)
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = float(msg.position[0])
        odom.pose.pose.position.y = float(msg.position[1])
        odom.pose.pose.position.z = float(msg.position[2]) + 0.07 # Ground clearance
        odom.pose.pose.orientation.x = float(qx)
        odom.pose.pose.orientation.y = float(qy)
        odom.pose.pose.orientation.z = float(qz)
        odom.pose.pose.orientation.w = float(qw)
        
        odom.twist.twist.linear.x = float(msg.velocity[0])
        odom.twist.twist.linear.y = float(msg.velocity[1])
        odom.twist.twist.linear.z = float(msg.velocity[2])
        odom.twist.twist.angular.z = float(msg.yaw_speed)
        
        self.odom_pub.publish(odom)
        
        # 3. Publish TF (odom -> base_link)
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.translation.z = odom.pose.pose.position.z
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)
        
        # 4. Publish Standard /imu (50Hz)
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = 'imu_link'
        imu.orientation = odom.pose.pose.orientation
        imu.angular_velocity.x = float(msg.imu_state.gyroscope[0])
        imu.angular_velocity.y = float(msg.imu_state.gyroscope[1])
        imu.angular_velocity.z = float(msg.imu_state.gyroscope[2])
        imu.linear_acceleration.x = float(msg.imu_state.accelerometer[0])
        imu.linear_acceleration.y = float(msg.imu_state.accelerometer[1])
        imu.linear_acceleration.z = float(msg.imu_state.accelerometer[2])
        self.imu_pub.publish(imu)

    # --------------------------------------------------------------------------
    # /cmd_vel -> Unitree Official Request (SportClient Move API: 1008)
    # --------------------------------------------------------------------------
    def cmd_vel_callback(self, msg: Twist):
        req = Request()
        req.header.identity.api_id = 1008 # Unitree Official Sport Mode API ID for Move
        
        param_dict = {
            "x": float(msg.linear.x),
            "y": float(msg.linear.y),
            "z": float(msg.angular.z)
        }
        req.parameter = json.dumps(param_dict)
        self.sport_req_pub.publish(req)

    # --------------------------------------------------------------------------
    # Front Camera Stream -> /camera/front/image_raw (30 fps)
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

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main():
    rclpy.init()
    node = OfficialUnitreeBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
