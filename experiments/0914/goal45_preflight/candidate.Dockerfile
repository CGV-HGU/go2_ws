# Locally tagged from the exact installed image ID; verify before rebuilding.
FROM escape-navigation:goal45-base-8a36ebd1
COPY candidate/motion.py candidate/action_executor_node.py /opt/s2e-robot-minimal/lib/python3.12/site-packages/s2e_vlm_robot/
COPY candidate/robot_escape.launch.py /opt/s2e-robot-minimal/share/s2e_vlm_robot/launch/robot_escape.launch.py
