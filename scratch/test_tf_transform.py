import rclpy
from rclpy.node import Node
import tf2_ros
import time

class TFTest(Node):
    def __init__(self):
        super().__init__('tf_test_scratch')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def test(self):
        print("Waiting for TF buffer to fill (3 seconds)...")
        time.sleep(3)
        
        frames = ['camera_gyro_optical_frame', 'camera_imu_optical_frame', 'camera_accel_optical_frame']
        for target in frames:
            try:
                trans = self.tf_buffer.lookup_transform('camera_link', target, rclpy.time.Time())
                print(f"✅ Success 'camera_link' -> '{target}': {trans.transform.translation}")
            except Exception as e:
                print(f"❌ Failed 'camera_link' -> '{target}': {e}")

def main():
    rclpy.init()
    node = TFTest()
    node.test()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
