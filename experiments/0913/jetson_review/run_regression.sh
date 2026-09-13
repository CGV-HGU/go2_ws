#!/usr/bin/env bash
set -eu
cd /home/unitree/s2e-vlm-async-framework-minimal
image=escape-navigation:paired-goals-20260913
run_group() {
 local label=$1
 shift
 docker run --rm --network none --name "s2e-postdrive-$label" --entrypoint bash \
  -e S2E_TEST_ROBOT_ESCAPE_LAUNCH=/opt/s2e-robot-minimal/share/s2e_vlm_robot/launch/robot_escape.launch.py -e ROS_DOMAIN_ID=214 -e CYCLONEDDS_URI=file:///work/config/cyclonedds-loopback.xml \
  -e S2E_TEST_ARTIFACT_DIR=/work/.local-data/jetson-post-drive-20260913/test-artifacts \
  -v /home/unitree/s2e-vlm-async-framework-minimal:/work -v /home/unitree/go2_ws_antarctica/cyclonedds.xml:/home/unitree/go2_ws_antarctica/cyclonedds.xml:ro -w /work "$image" \
  -lc 'source /opt/ros/jazzy/setup.bash && source /opt/s2e-robot-minimal/setup.bash && export PYTHONPATH=/work:/work/src/s2e_vlm_nodes/test:/work/src/s2e_vlm_habitat:/work/scripts:$PYTHONPATH && python3 /work/.local-data/jetson-post-drive-20260913/run_installed_tests.py "$@" -q --tb=short' test "$@" \
  > ".local-data/jetson-post-drive-20260913/installed-$label.log" 2>&1
}
# Separate processes because the older simulator graph changes ROS_DOMAIN_ID.
# Imports use the installed candidate runtime, not source bind overrides.
case "${1:-all}" in
  all|runtime) run_group runtime src/s2e_vlm_nodes/test src/s2e_vlm_robot/test --ignore=src/s2e_vlm_nodes/test/test_ros_mock_graph.py ;;&
  all|operator) run_group operator scripts/test_fixed_start_goals.py scripts/test_robot_pointgoal_trial.py scripts/test_robot_prepared_trial.py scripts/test_capture_robot_goal.py scripts/test_robot_episode_archive.py scripts/test_robot_analysis_frames.py scripts/test_prepare_robot_map_goals.py scripts/test_robot_map_session.py ;;&
  all|simulator) run_group simulator src/s2e_vlm_nodes/test/test_ros_mock_graph.py ;;&
  nodes) run_group nodes src/s2e_vlm_nodes/test --ignore=src/s2e_vlm_nodes/test/test_ros_mock_graph.py ;;
  robot) run_group robot src/s2e_vlm_robot/test ;;
  failure-recheck) run_group failure-recheck src/s2e_vlm_nodes/test/test_s2e_async_fault_injection.py::S2EAsyncFaultInjectionTest::test_worker_disconnect_timeout_and_malformed_results_fail_closed ;;
esac
