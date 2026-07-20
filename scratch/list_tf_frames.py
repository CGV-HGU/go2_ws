import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener_scratch')
        self.tf_frames = set()
        self.tf_static_frames = set()
        self.sub_tf = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.sub_tf_static = self.create_subscription(TFMessage, '/tf_static', self.tf_static_callback, 10)

    def tf_callback(self, msg):
        for transform in msg.transforms:
            self.tf_frames.add((transform.header.frame_id, transform.child_frame_id))

    def tf_static_callback(self, msg):
        for transform in msg.transforms:
            self.tf_static_frames.add((transform.header.frame_id, transform.child_frame_id))

def main():
    rclpy.init()
    node = TFListener()
    print("Listening to /tf and /tf_static for 5 seconds...")
    import time
    start = time.time()
    while time.time() - start < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    print("\n--- Active frames in /tf ---")
    for parent, child in sorted(list(node.tf_frames)):
        print(f"  {parent} -> {child}")
        
    print("\n--- Active frames in /tf_static ---")
    for parent, child in sorted(list(node.tf_static_frames)):
        print(f"  {parent} -> {child}")
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
