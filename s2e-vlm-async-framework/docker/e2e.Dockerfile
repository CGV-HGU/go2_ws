FROM s2e-onnx-runtime-base:latest

WORKDIR /workspace
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility

CMD ["source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && ros2 run s2e_vlm_nodes e2e_node"]
