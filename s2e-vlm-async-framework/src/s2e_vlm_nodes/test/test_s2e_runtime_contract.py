import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


class S2ERuntimeContractTest(unittest.TestCase):
    def test_e2e_runtime_declares_s2e_backend_switch_and_image_context_status(self):
        runtime = (ROOT / "s2e_vlm_nodes" / "s2e_vlm_nodes" / "ros_mock_runtime.py").read_text(encoding="utf-8")

        self.assertIn('os.environ.get("E2E_BACKEND", "mock")', runtime)
        self.assertIn("S2EFrameContext", runtime)
        self.assertIn("ros_rgb8_to_chw_float", runtime)
        self.assertIn("WAITING_IMAGE_CONTEXT", runtime)
        backend = (ROOT / "s2e_vlm_core" / "s2e_vlm_core" / "s2e_backend.py").read_text(encoding="utf-8")
        self.assertIn("S2E_FRESH", backend)

    def test_vlm_runtime_declares_qwen_api_backend_without_replacing_agentic_node(self):
        runtime = (ROOT / "s2e_vlm_nodes" / "s2e_vlm_nodes" / "ros_mock_runtime.py").read_text(encoding="utf-8")

        self.assertIn('os.environ.get("VLM_BACKEND", "mock")', runtime)
        self.assertIn("VLM_API_URL", runtime)
        self.assertIn("VLM_API_TIMEOUT_S", runtime)
        self.assertIn("VLM_API_MAX_RETRIES", runtime)
        self.assertIn("_call_qwen_api", runtime)
        self.assertIn("_extract_qwen_content", runtime)
        self.assertIn("API_TIMEOUT", runtime)


if __name__ == "__main__":
    unittest.main()
