FROM s2e-ros-base:latest

WORKDIR /workspace

CMD ["bash", "-lc", "source /workspace/install/setup.bash && ros2 launch s2e_vlm_bringup robot_side.launch.py"]
