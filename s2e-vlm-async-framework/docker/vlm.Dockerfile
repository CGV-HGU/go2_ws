FROM s2e-ros-base:latest

WORKDIR /workspace

CMD ["bash", "-lc", "source /workspace/install/setup.bash && ros2 run s2e_vlm_nodes vlm_node"]
