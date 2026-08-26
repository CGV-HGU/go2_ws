#!/usr/bin/env python3
"""
Unitree Go2 Built-in Front Ultra-Wide RGB Camera ROS 2 Publisher
================================================================
Non-blocking GStreamer RTP multicast capture (230.1.1.1:1720) from Go2 head.
Publishes sensor_msgs/Image to /camera/front/image_raw (30 fps)
along with synchronized sensor_msgs/CameraInfo to /camera/front/camera_info.
"""

import os
import sys
import threading
import time
import socket
import struct

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
        
        # Standard Reliable QoS for Universal ROS 2 CLI & SLAM Compatibility
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
        self.get_logger().info('📷 Go2 Front Camera Node Started (Connecting to 230.1.1.1:1720 in background)...')
        
        # Start Non-blocking Capture Thread
        self.thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.thread.start()

    def is_rtp_camera_streaming(self, ip="230.1.1.1", port=1720, timeout=0.5):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('', port))
            mreq = struct.pack("4sl", socket.inet_aton(ip), socket.INADDR_ANY)
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
            sock.settimeout(timeout)
            data, _ = sock.recvfrom(1024)
            sock.close()
            return len(data) > 0
        except Exception:
            return False

    def capture_loop(self):
        pipeline = (
            'udpsrc address=230.1.1.1 port=1720 multicast-group=230.1.1.1 auto-multicast=true timeout=1000000000 ! '
            'application/x-rtp, media=video, clock-rate=90000, payload=96, encoding-name=H264 ! '
            'rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink drop=true max-buffers=1'
        )

        while rclpy.ok() and self.running:
            if self.cap is None or not self.cap.isOpened():
                if self.is_rtp_camera_streaming(timeout=0.3):
                    self.get_logger().info('🟢 Live RTP Camera Stream Detected! Opening GStreamer pipeline...')
                    self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
                else:
                    # Provide standby FPV frame so RTAB-Map approx_sync never blocks!
                    syn_frame = np.full((720, 1280, 3), 45, dtype=np.uint8)
                    cv2.putText(syn_frame, "Unitree Go2 Front Camera (Standby)", (380, 360),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 200), 2, cv2.LINE_AA)
                    self._publish_frame(syn_frame)
                    time.sleep(0.1) # 10Hz heartbeat
                    continue

            ret, frame = self.cap.read()
            if ret and frame is not None:
                self._publish_frame(frame)
            else:
                if self.cap:
                    self.cap.release()
                self.cap = None
                time.sleep(0.1)

    def _publish_frame(self, frame):
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
        if hasattr(self, 'cap') and self.cap is not None and self.cap.isOpened():
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
