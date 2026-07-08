FROM s2e-ros-base:latest

WORKDIR /workspace
COPY README.md docs ./docs/

CMD ["bash", "-lc", "source /workspace/install/setup.bash && ros2 launch s2e_vlm_bringup single_pc_mock.launch.py"]
