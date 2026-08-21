#!/usr/bin/env python3
"""
Unitree Go2 Built-in Front Ultra-Wide RGB Camera ROS 2 Publisher
================================================================
Captures H.264 RTP multicast stream (230.1.1.1:1720) from Go2 head
and publishes standard sensor_msgs/Image to /camera/front/image_raw (30 fps)
along with synchronized sensor_msgs/CameraInfo to /camera/front/camera_info.
Thread-safe, non-blocking capture loop with SensorData QoS.
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
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class Go2FrontCameraPublisher(Node):
    def __init__(self):
        super().__init__('go2_front_camera_publisher')
        
        # SensorData QoS (Best-Effort, Volatile) for real-time video
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.image_pub = self.create_publisher(Image, '/camera/front/image_raw', sensor_qos)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/front/camera_info', sensor_qos)
        self.bridge = CvBridge()
        
        # Go2 내장 카메라 H.264 하드웨어 디코딩 GStreamer 파이프라인
        pipeline = (
            'udpsrc address=230.1.1.1 port=1720 multicast-group=230.1.1.1 auto-multicast=true ! '
            'application/x-rtp, media=video, clock-rate=90000, payload=96, encoding-name=H264 ! '
            'rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink drop=true max-buffers=1'
        )
        
        self.get_logger().info('Connecting to Go2 Built-in Front Camera GStreamer stream...')
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open GStreamer pipeline for Go2 front camera!')
            self.running = False
        else:
            self.get_logger().info('✅ Go2 Built-in Front Camera Connected! Publishing Image & CameraInfo (30fps)')
            self.running = True
            self.thread = threading.Thread(target=self.capture_loop, daemon=True)
            self.thread.start()

    def capture_loop(self):
        while rclpy.ok() and self.running:
            if not self.cap.isOpened():
                time.sleep(0.1)
                continue
            ret, frame = self.cap.read()
            if ret and frame is not None:
                now = self.get_clock().now().to_msg()
                
                # 1. Publish Image
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                img_msg.header.stamp = now
                img_msg.header.frame_id = 'camera_link'
                self.image_pub.publish(img_msg)
                
                # 2. Publish Synchronized CameraInfo
                info = CameraInfo()
                info.header.stamp = now
                info.header.frame_id = 'camera_link'
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
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
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
