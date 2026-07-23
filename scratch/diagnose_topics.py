#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

class TopicDiagnoser(Node):
    def __init__(self):
        super().__init__('topic_diagnoser')
        
    def diagnose(self):
        self.get_logger().info("Diagnosing ROS 2 topics...")
        
        # Get all topic names and types
        topic_names_and_types = self.get_topic_names_and_types()
        self.get_logger().info(f"Total topics found: {len(topic_names_and_types)}")
        
        target_topics = ['/camera/imu', '/imu/data_raw', '/imu/data', '/rtabmap/imu']
        
        for topic_name, topic_types in topic_names_and_types:
            if topic_name in target_topics or 'imu' in topic_name.lower():
                self.get_logger().info(f"\nTopic: {topic_name} (Types: {topic_types})")
                
                # Get publisher info
                try:
                    pubs = self.get_publishers_info_by_topic(topic_name)
                    self.get_logger().info(f"  Publishers ({len(pubs)}):")
                    for p in pubs:
                        self.get_logger().info(f"    - Node: {p.node_name}/{p.node_namespace}")
                        self.get_logger().info(f"      QoS: Reliability={p.qos_profile.reliability}, Durability={p.qos_profile.durability}")
                except Exception as e:
                    self.get_logger().error(f"    Error getting publishers: {e}")
                    
                # Get subscriber info
                try:
                    subs = self.get_subscriptions_info_by_topic(topic_name)
                    self.get_logger().info(f"  Subscribers ({len(subs)}):")
                    for s in subs:
                        self.get_logger().info(f"    - Node: {s.node_name}/{s.node_namespace}")
                        self.get_logger().info(f"      QoS: Reliability={s.qos_profile.reliability}, Durability={s.qos_profile.durability}")
                except Exception as e:
                    self.get_logger().error(f"    Error getting subscribers: {e}")

def main():
    rclpy.init()
    node = TopicDiagnoser()
    node.diagnose()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
