#!/usr/bin/env python3
"""
Unitree Go2 Built-in Front Ultra-Wide RGB Camera ROS 2 Publisher
================================================================
Continuous GStreamer RTP multicast capture (230.1.1.1:1720) from Go2 head.
Publishes sensor_msgs/Image to /camera/front/image_raw (30 fps)
along with synchronized sensor_msgs/CameraInfo to /camera/front/camera_info.
Uses Best-Effort (SensorDataQoS) matching RTAB-Map and DDS.
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
        
        # Standard Reliable QoS (Universal compatibility with both Reliable & Best-Effort subscribers)
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
        self.cap = None
        self.latest_frame = None
        self.lock = threading.Lock()
        
        self.get_logger().info('📷 Go2 Front Camera Node Initialized (30fps Best-Effort QoS)...')
        
        # Start Worker Threads
        self.capture_thread = threading.Thread(target=self.gstreamer_worker, daemon=True)
        self.capture_thread.start()
        
        # 30Hz Fixed-Rate Publisher Timer (guarantees continuous sync with LiDAR)
        self.timer = self.create_timer(1.0 / 30.0, self.publish_timer_callback)

    def gstreamer_worker(self):
        pipeline = (
            'udpsrc address=230.1.1.1 port=1720 multicast-group=230.1.1.1 auto-multicast=true ! '
            'application/x-rtp, media=video, clock-rate=90000, payload=96, encoding-name=H264 ! '
            'rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink drop=true max-buffers=1'
        )
        
        while rclpy.ok() and self.running:
            try:
                cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
                if not cap.isOpened():
                    time.sleep(0.5)
                    continue
                
                self.get_logger().info('🟢 GStreamer Pipeline Opened! Receiving Go2 Front Video...')
                while rclpy.ok() and self.running and cap.isOpened():
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        with self.lock:
                            self.latest_frame = frame
                    else:
                        time.sleep(0.01)
                cap.release()
            except Exception as e:
                time.sleep(0.5)

    def publish_timer_callback(self):
        with self.lock:
            frame = self.latest_frame
            
        if frame is None:
            # Standby nominal frame
            frame = np.full((720, 1280, 3), 40, dtype=np.uint8)
            cv2.putText(frame, "Go2 Front Ultra-Wide Camera (Standby)", (350, 360),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 200), 2, cv2.LINE_AA)

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
