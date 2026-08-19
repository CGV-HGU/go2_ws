#!/usr/bin/env python3
"""
Unitree Go2 Built-in Front Ultra-Wide RGB Camera ROS 2 Publisher
================================================================
Captures H.264 RTP multicast stream (230.1.1.1:1720) from Go2 head
and publishes standard sensor_msgs/Image to /camera/front/image_raw (30 fps).
"""

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Go2FrontCameraPublisher(Node):
    def __init__(self):
        super().__init__('go2_front_camera_publisher')
        
        # ICRA 2026 ESCAPE-Nav 표준 토픽명으로 발행
        self.pub = self.create_publisher(Image, '/camera/front/image_raw', 10)
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
        else:
            self.get_logger().info('✅ Go2 Built-in Front Camera Connected! Publishing to /camera/front/image_raw (30fps)')
            self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        if not self.cap.isOpened():
            return
        ret, frame = self.cap.read()
        if ret and frame is not None:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_link'
            self.pub.publish(msg)

    def destroy_node(self):
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
