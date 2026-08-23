#!/usr/bin/env python3
"""
========================================================================================
🔍 [Unitree Go2] Official Robot State & Service Switch Inspector (/api/robot_state)
========================================================================================
Uses Unitree Official DDS API (ROBOT_STATE_API_ID = 1001, 1003) to:
  1. Query all internal robot services (ServiceList)
  2. Switch on/off internal LiDAR / Obstacle Avoidance / Sport services (ServiceSwitch)
  3. 100% programmatic - Zero smartphone app dependency!
========================================================================================
"""

import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from unitree_api.msg import Request, Response

ROBOT_STATE_API_ID_SERVICE_SWITCH = 1001
ROBOT_STATE_API_ID_SET_REPORT_FREQ = 1002
ROBOT_STATE_API_ID_SERVICE_LIST = 1003

class Go2ServiceInspector(Node):
    def __init__(self):
        super().__init__('go2_service_inspector')
        
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        self.req_pub = self.create_publisher(Request, '/api/robot_state/request', qos)
        self.res_sub = self.create_subscription(Response, '/api/robot_state/response', self.on_response, qos)
        
        self.pending_id = None
        self.response_received = False
        self.response_data = None
        
        print("=" * 76)
        print(" 🔍 [Go2 Service Inspector] Querying Official Robot State API (/api/robot_state)...")
        print("=" * 76)

    def query_service_list(self, timeout_sec=3.0):
        req = Request()
        req_id = int(time.time() * 1e9)
        self.pending_id = req_id
        
        req.header.identity.id = req_id
        req.header.identity.api_id = ROBOT_STATE_API_ID_SERVICE_LIST
        req.parameter = "{}"
        
        self.response_received = False
        self.response_data = None
        
        # Publish request
        time.sleep(0.2)
        self.req_pub.publish(req)
        print(f"  • Sent ServiceList request (ID: {req_id}). Waiting for response...")
        
        start_t = time.time()
        while time.time() - start_t < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.response_received:
                return self.response_data
        
        print("  ⚠️ ServiceList query timed out (Mainboard /api/robot_state not answering).")
        return None

    def switch_service(self, service_name, state=1, timeout_sec=3.0):
        """state: 1 = ON, 0 = OFF"""
        req = Request()
        req_id = int(time.time() * 1e9)
        self.pending_id = req_id
        
        req.header.identity.id = req_id
        req.header.identity.api_id = ROBOT_STATE_API_ID_SERVICE_SWITCH
        param = {"name": service_name, "switch": state}
        req.parameter = json.dumps(param)
        
        self.response_received = False
        self.response_data = None
        
        time.sleep(0.2)
        self.req_pub.publish(req)
        print(f"  • Sent ServiceSwitch request for '{service_name}' -> {'ON' if state else 'OFF'}...")
        
        start_t = time.time()
        while time.time() - start_t < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.response_received:
                return self.response_data
        
        print(f"  ⚠️ ServiceSwitch for '{service_name}' timed out.")
        return None

    def on_response(self, msg: Response):
        if self.pending_id and msg.header.identity.id == self.pending_id:
            self.response_received = True
            try:
                self.response_data = json.loads(msg.data)
            except Exception:
                self.response_data = msg.data
            
            print("=" * 76)
            print(f" 🟢 [RESPONSE RECEIVED] Status: {msg.header.status.code}")
            print(f" 📄 Data: {json.dumps(self.response_data, indent=2)}")
            print("=" * 76)

def main():
    rclpy.init()
    inspector = Go2ServiceInspector()
    
    # 1. Query all active services on Go2 mainboard
    services = inspector.query_service_list()
    
    # 2. If 'lidar' or 'utlidar' is found, ensure it is switched ON
    if services and isinstance(services, list):
        for s in services:
            name = s.get('name', '')
            status = s.get('status', 0)
            print(f"  -> Service: {name:20s} | Status: {'ACTIVE 🟢' if status == 1 else 'OFF ⚪'}")
            if 'lidar' in name.lower() and status == 0:
                print(f"  🚀 Activating {name} via official ROS 2 API...")
                inspector.switch_service(name, 1)

    inspector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
