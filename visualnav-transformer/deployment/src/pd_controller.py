import numpy as np
import yaml
from typing import Tuple

# ROS
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Bool

from topic_names import (WAYPOINT_TOPIC, 
			 			REACHED_GOAL_TOPIC)
from ros_data import ROSData
from utils import clip_angle

# CONSTS
CONFIG_PATH = "../config/robot.yaml"
with open(CONFIG_PATH, "r") as f:
	robot_config = yaml.safe_load(f)
MAX_V = robot_config["max_v"]
MAX_W = robot_config["max_w"]
VEL_TOPIC = robot_config["vel_navi_topic"]
DT = 1/robot_config["frame_rate"]
RATE = 9
EPS = 1e-8
WAYPOINT_TIMEOUT = 1 # seconds # TODO: tune this
FLIP_ANG_VEL = np.pi/4

# GLOBALS
vel_msg = Twist()
waypoint = ROSData(WAYPOINT_TIMEOUT, name="waypoint")
reached_goal = False
reverse_mode = False
current_yaw = None

def clip_angle(theta) -> float:
	"""Clip angle to [-pi, pi]"""
	theta %= 2 * np.pi
	if -np.pi < theta < np.pi:
		return theta
	return theta - 2 * np.pi
      

def pd_controller(waypoint: np.ndarray) -> Tuple[float, float, float]:
	"""3-DOF Omnidirectional PD controller for Go2 quadruped (vx, vy, w)"""
	assert len(waypoint) == 2 or len(waypoint) == 4, "waypoint must be a 2D or 4D vector"
	if len(waypoint) == 2:
		dx, dy = waypoint
	else:
		dx, dy, hx, hy = waypoint

	if len(waypoint) == 4 and np.abs(dx) < EPS and np.abs(dy) < EPS:
		v_x = 0.0
		v_y = 0.0
		w = clip_angle(np.arctan2(hy, hx)) / DT
	elif np.abs(dx) < EPS:
		v_x = 0.0
		v_y = np.clip(dy / DT, -0.2, 0.2) # Holonomic strafing velocity
		w = np.sign(dy) * np.pi / (2 * DT)
	else:
		v_x = dx / DT
		v_y = np.clip(dy / (2 * DT), -0.2, 0.2) # Gentle 3-DOF lateral velocity for smooth cornering
		w = np.arctan2(dy, dx) / DT

	v_x = np.clip(v_x, 0, MAX_V)
	w = np.clip(w, -MAX_W, MAX_W)
	return v_x, v_y, w


def callback_drive(waypoint_msg: Float32MultiArray):
	"""Callback function for the waypoint subscriber"""
	global vel_msg
	print("seting waypoint")
	waypoint.set(waypoint_msg.data)
	
	
def callback_reached_goal(reached_goal_msg: Bool):
	"""Callback function for the reached goal subscriber"""
	global reached_goal
	reached_goal = reached_goal_msg.data


def main():
	global vel_msg, reverse_mode
	rospy.init_node("PD_CONTROLLER", anonymous=False)
	waypoint_sub = rospy.Subscriber(WAYPOINT_TOPIC, Float32MultiArray, callback_drive, queue_size=1)
	reached_goal_sub = rospy.Subscriber(REACHED_GOAL_TOPIC, Bool, callback_reached_goal, queue_size=1)
	vel_out = rospy.Publisher(VEL_TOPIC, Twist, queue_size=1)
	rate = rospy.Rate(RATE)
	print("Registered with master node. Waiting for waypoints...")
	while not rospy.is_shutdown():
		vel_msg = Twist()
		if reached_goal:
			vel_out.publish(vel_msg)
			print("Reached goal! Stopping...")
			return
		elif waypoint.is_valid(verbose=True):
			v_x, v_y, w = pd_controller(waypoint.get())
			if reverse_mode:
				v_x *= -1
				v_y *= -1
			vel_msg.linear.x = v_x
			vel_msg.linear.y = v_y
			vel_msg.angular.z = w
			print(f"publishing 3-DOF vel: vx={v_x:.2f}, vy={v_y:.2f}, w={w:.2f}")
		vel_out.publish(vel_msg)
		rate.sleep()
	

if __name__ == '__main__':
	main()
