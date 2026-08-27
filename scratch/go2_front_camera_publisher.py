#!/usr/bin/env python3
"""
Unitree Go2 Built-in Front Ultra-Wide RGB Camera ROS 2 Publisher
================================================================
Jetson Hardware Accelerated GStreamer RTP multicast capture (230.1.1.1:1720) from Go2 head.
Publishes sensor_msgs/Image to /camera/front/image_raw (30 fps)
along with synchronized sensor_msgs/CameraInfo to /camera/front/camera_info.
"""

import os
import sys
import threading
import time

# Ensure OpenCV libraries are always loaded
opencv_lib = '/home/unitree/opencv_build/opencv/build/lib'
if opencv_lib not in os.environ.get('LD_LIBRARY_PATH', ''):
    os.environ['LD_LIBRARY_PATH'] = f"{opencv_lib}:/usr/local/lib:" + os.environ.get('LD_LIBRARY_PATH', '')

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class Go2FrontCameraPublisher(Node):
    def __init__(self):
        super().__init__('go2_front_camera_publisher')
        
        # Standard Reliable QoS for universal ROS 2 compatibility
        camera_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.image_pub = self.create_publisher(Image, '/camera/front/image_raw', camera_qos)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/front/camera_info', camera_qos)
        self.bridge = CvBridge()
        
        self.running = True
        self.has_received_live_frame = False
        
        self.get_logger().info('📷 Go2 Front Camera Node Initialized (Jetson NVDEC Hardware Stream)...')
        
        # Start GStreamer Worker Thread
        self.capture_thread = threading.Thread(target=self.gstreamer_worker, daemon=True)
        self.capture_thread.start()

    def gstreamer_worker(self):
        pipeline = (
            'udpsrc port=1720 multicast-group=230.1.1.1 auto-multicast=true ! '
            'application/x-rtp, media=video, encoding-name=H264 ! '
            'rtph264depay ! h264parse ! nvv4l2decoder ! nvvidconv ! '
            'video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink drop=true max-buffers=1'
        )
        
        while rclpy.ok() and self.running:
            try:
                cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
                if not cap.isOpened():
                    time.sleep(0.5)
                    continue
                
                self.get_logger().info('🟢 Live Camera Stream Connected! Streaming real 1280x720 FPV frames...')
                while rclpy.ok() and self.running and cap.isOpened():
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        self.has_received_live_frame = True
                        self.publish_frame(frame)
                    else:
                        time.sleep(0.005)
                cap.release()
            except Exception as e:
                time.sleep(0.5)

    def publish_frame(self, frame):
        now = self.get_clock().now().to_msg()
        
        # 1. Publish Image
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = now
        # ROS CameraInfo/Image frames use the optical convention: +z forward,
        # +x right, +y down. The launch file publishes base_link -> this frame.
        img_msg.header.frame_id = 'camera_optical_frame'
        self.image_pub.publish(img_msg)
        
        # 2. Publish Synchronized CameraInfo
        info = CameraInfo()
        info.header.stamp = now
        info.header.frame_id = 'camera_optical_frame'
        info.width = 1280
        info.height = 720
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [600.0, 0.0, 640.0, 0.0, 600.0, 360.0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [600.0, 0.0, 640.0, 0.0, 0.0, 600.0, 360.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)

    def destroy_node(self):
        self.running = False
        super().destroy_node()

def main():
    rclpy.init()
    node = Go2FrontCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
