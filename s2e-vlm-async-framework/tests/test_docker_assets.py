import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _compose_service_block(service_name: str) -> str:
    lines = (ROOT / "compose.yaml").read_text(encoding="utf-8").splitlines()
    start_marker = f"  {service_name}:"
    start_index = lines.index(start_marker)
    block_lines = [lines[start_index]]
    for line in lines[start_index + 1 :]:
        if line.startswith("  ") and not line.startswith("    "):
            break
        block_lines.append(line)
    return "\n".join(block_lines)


class DockerAssetsTest(unittest.TestCase):
    def test_required_dockerfiles_exist(self):
        for file_name in [
            "ros-base.Dockerfile",
            "dev-mock.Dockerfile",
            "robot.Dockerfile",
            "gpu-inference-base.Dockerfile",
            "onnx-runtime-base.Dockerfile",
            "vlm.Dockerfile",
            "e2e.Dockerfile",
        ]:
            with self.subTest(file_name=file_name):
                self.assertTrue((ROOT / "docker" / file_name).is_file())

    def test_compose_declares_required_profiles_and_gpu_only_for_model_services(self):
        compose = (ROOT / "compose.yaml").read_text(encoding="utf-8")

        for profile in ["single_pc_mock", "single_pc_split", "robot_side", "external_gpu", "vlm_only", "e2e_only"]:
            self.assertIn(profile, compose)
        self.assertIn("s2e-onnx-runtime-base", compose)
        self.assertIn("NVIDIA_DRIVER_CAPABILITIES: compute,utility", compose)
        self.assertNotIn("s2e-robot\n    deploy:", compose)

    def test_compose_maps_three_way_split_profiles_to_only_their_services(self):
        expected_profiles = {
            "robot-core": ["single_pc_split", "robot_side"],
            "vlm": ["single_pc_split", "external_gpu", "vlm_only"],
            "e2e": ["single_pc_split", "external_gpu", "e2e_only"],
        }
        forbidden_profiles = {
            "robot-core": ["external_gpu", "vlm_only", "e2e_only"],
            "vlm": ["robot_side", "e2e_only"],
            "e2e": ["robot_side", "vlm_only"],
        }

        for service_name, profiles in expected_profiles.items():
            service_block = _compose_service_block(service_name)
            for profile in profiles:
                with self.subTest(service=service_name, profile=profile):
                    self.assertIn(f'"{profile}"', service_block)

        for service_name, profiles in forbidden_profiles.items():
            service_block = _compose_service_block(service_name)
            for profile in profiles:
                with self.subTest(service=service_name, profile=profile):
                    self.assertNotIn(f'"{profile}"', service_block)

    def test_docs_document_three_machine_vlm_and_e2e_split_commands(self):
        readme = (ROOT / "README.md").read_text(encoding="utf-8")
        testing = (ROOT / "docs" / "testing.md").read_text(encoding="utf-8")
        requirements = (ROOT / "docs" / "requirements.md").read_text(encoding="utf-8")

        for document in [readme, testing]:
            with self.subTest(document=document[:40]):
                self.assertIn("docker compose --profile vlm_only up vlm", document)
                self.assertIn("docker compose --profile e2e_only up e2e", document)
                self.assertIn("docker compose --profile robot_side --profile vlm_only --profile e2e_only up robot-core vlm e2e", document)

        self.assertIn("vlm_only", requirements)
        self.assertIn("e2e_only", requirements)

    def test_env_example_documents_ros_and_gpu_variables(self):
        env_example = (ROOT / ".env.example").read_text(encoding="utf-8")

        for key in ["ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_LOCALHOST_ONLY", "GPU_DEVICE_ID", "S2E_SENSOR_CONFIG_DIR"]:
            self.assertIn(key, env_example)

    def test_compose_passes_sensor_config_override_to_ros_services(self):
        compose = (ROOT / "compose.yaml").read_text(encoding="utf-8")

        self.assertIn("S2E_SENSOR_CONFIG_DIR: ${S2E_SENSOR_CONFIG_DIR:-}", compose)

    def test_onnx_runtime_base_uses_same_ros_distro_as_cpu_containers(self):
        onnx_base = (ROOT / "docker" / "onnx-runtime-base.Dockerfile").read_text(encoding="utf-8")
        vlm = (ROOT / "docker" / "vlm.Dockerfile").read_text(encoding="utf-8")
        e2e = (ROOT / "docker" / "e2e.Dockerfile").read_text(encoding="utf-8")

        self.assertIn("ubuntu24.04", onnx_base)
        self.assertIn("ros-jazzy-ros-base", onnx_base)
        self.assertIn("onnxruntime-gpu==1.23.2", onnx_base)
        self.assertNotIn("download.pytorch.org", onnx_base)
        self.assertNotIn("torch", onnx_base)
        self.assertNotIn("ros-humble", onnx_base)
        self.assertIn("/opt/ros/jazzy/setup.bash", e2e)

    def test_optional_gpu_inference_base_keeps_pytorch_for_checkpoint_models(self):
        gpu_base = (ROOT / "docker" / "gpu-inference-base.Dockerfile").read_text(encoding="utf-8")

        self.assertIn("download.pytorch.org/whl/cu128", gpu_base)
        self.assertIn("torch", gpu_base)
        self.assertIn("safetensors", gpu_base)
        self.assertIn("pyyaml", gpu_base)

    def test_compose_wires_s2e_backend_model_mount_and_vlm_api_config(self):
        compose = (ROOT / "compose.yaml").read_text(encoding="utf-8")
        env_example = (ROOT / ".env.example").read_text(encoding="utf-8")
        gitignore = (ROOT / ".gitignore").read_text(encoding="utf-8")
        e2e_block = _compose_service_block("e2e")
        vlm_block = _compose_service_block("vlm")

        self.assertIn("nav_model_zoo/", gitignore)
        for key in ["E2E_BACKEND", "E2E_MODEL_PATH", "VLM_BACKEND", "VLM_API_URL", "VLM_API_TIMEOUT_S", "VLM_API_MAX_RETRIES"]:
            with self.subTest(key=key):
                self.assertIn(f"{key}=", env_example)
                self.assertIn(f"{key}: ${{{key}:-", compose)

        self.assertIn("${S2E_NAV_MODEL_ZOO_DIR:-./nav_model_zoo}:/models/s2e:ro", e2e_block)
        self.assertIn("S2E_NAV_MODEL_ZOO_DIR=./nav_model_zoo", env_example)
        self.assertIn("VLM_API_URL", vlm_block)

    def test_vlm_runtime_is_api_only_without_gpu_or_model_mount(self):
        vlm = (ROOT / "docker" / "vlm.Dockerfile").read_text(encoding="utf-8")
        vlm_block = _compose_service_block("vlm")

        self.assertIn("FROM s2e-ros-base:latest", vlm)
        self.assertIn('CMD ["bash", "-lc", "source /workspace/install/setup.bash && ros2 run s2e_vlm_nodes vlm_node"]', vlm)
        self.assertNotIn("s2e-gpu-inference-base", vlm)
        self.assertNotIn("s2e-onnx-runtime-base", vlm)
        self.assertNotIn("deploy:", vlm_block)
        self.assertNotIn("capabilities: [gpu]", vlm_block)
        self.assertNotIn("VLM_MODEL_PATH", vlm_block)
        self.assertNotIn("/models/s2e", vlm_block)

    def test_e2e_runtime_uses_onnx_base_not_pytorch_base(self):
        e2e = (ROOT / "docker" / "e2e.Dockerfile").read_text(encoding="utf-8")
        compose = (ROOT / "compose.yaml").read_text(encoding="utf-8")

        self.assertIn("FROM s2e-onnx-runtime-base:latest", e2e)
        self.assertNotIn("FROM s2e-gpu-inference-base", e2e)
        self.assertIn("s2e-onnx-runtime-base", compose)

    def test_compose_mounts_repo_artifacts_for_visualizer_outputs(self):
        compose = (ROOT / "compose.yaml").read_text(encoding="utf-8")
        gitignore = (ROOT / ".gitignore").read_text(encoding="utf-8")

        self.assertIn("${S2E_ARTIFACT_DIR:-./artifacts}:/artifacts", compose)
        self.assertIn("S2E_TEST_ARTIFACT_DIR: ${S2E_TEST_ARTIFACT_DIR:-/artifacts/visualizer}", compose)
        self.assertIn("artifacts/", gitignore)

    def test_compose_passes_mock_runtime_parameters_to_split_services(self):
        compose = (ROOT / "compose.yaml").read_text(encoding="utf-8")
        env_example = (ROOT / ".env.example").read_text(encoding="utf-8")

        required_runtime_vars = [
            "S2E_MOCK_CAMERA_PERIOD_S",
            "S2E_MOCK_CAMERA_MODE",
            "S2E_MOCK_VLM_PERIOD_S",
            "S2E_MOCK_VLM_SCENARIOS",
            "S2E_MOCK_E2E_PERIOD_S",
            "S2E_MOCK_DEBUG_VISUALIZER_PERIOD_S",
            "S2E_MOCK_ARTIFACT_SAVE_PERIOD_S",
            "S2E_MOCK_ARTIFACT_DURATION_S",
        ]
        for variable in required_runtime_vars:
            with self.subTest(variable=variable):
                self.assertIn(f"{variable}: ${{{variable}:-", compose)
                self.assertIn(f"{variable}=", env_example)

    def test_bringup_installs_per_sensor_calibration_configs(self):
        setup_py = (ROOT / "src" / "s2e_vlm_bringup" / "setup.py").read_text(encoding="utf-8")

        self.assertIn("config/sensors/*.yaml", setup_py)
        for sensor_name in ["camera", "lidar", "imu"]:
            with self.subTest(sensor_name=sensor_name):
                self.assertTrue((ROOT / "src" / "s2e_vlm_bringup" / "config" / "sensors" / f"{sensor_name}.yaml").is_file())

    def test_visualizer_runtime_dependencies_are_explicit(self):
        package_xml = (ROOT / "src" / "s2e_vlm_nodes" / "package.xml").read_text(encoding="utf-8")
        ros_base = (ROOT / "docker" / "ros-base.Dockerfile").read_text(encoding="utf-8")
        gpu_base = (ROOT / "docker" / "gpu-inference-base.Dockerfile").read_text(encoding="utf-8")

        for dependency in ["python3-numpy", "python3-opencv"]:
            with self.subTest(dependency=dependency):
                self.assertIn(dependency, package_xml)
                self.assertIn(dependency, ros_base)
                self.assertIn(dependency, gpu_base)

    def test_visualizer_mp4_uses_ffmpeg_h264_encoding(self):
        ros_base = (ROOT / "docker" / "ros-base.Dockerfile").read_text(encoding="utf-8")
        runtime = (ROOT / "src" / "s2e_vlm_nodes" / "s2e_vlm_nodes" / "ros_mock_runtime.py").read_text(encoding="utf-8")

        self.assertIn("ffmpeg", ros_base)
        self.assertIn("libx264", runtime)
        self.assertIn("yuv420p", runtime)


if __name__ == "__main__":
    unittest.main()
