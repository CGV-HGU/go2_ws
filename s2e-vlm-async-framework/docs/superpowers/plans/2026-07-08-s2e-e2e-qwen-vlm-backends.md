# S2E e2e and Qwen VLM Backends Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add real-model runtime wiring for S2E-based e2e inference and Qwen3-VL API-backed VLM orchestration while preserving deterministic mock behavior.

**Architecture:** `e2e_node` keeps the mock planner by default, but can switch to an S2E backend that buffers 11 RGB frames, builds `(1, 11, 3, 256, 256)` float32 input, calls S2E trajectory inference, and publishes the existing `Trajectory2D` contract. `vlm_node` remains a ROS-aware agentic orchestrator and API client; Qwen3-VL 32B Thinking runs in a separate model server process/service, and the ROS node owns parsing, schema validation, refinement, safety gates, heartbeat, and degraded status.

**Tech Stack:** ROS 2 Jazzy, Docker Compose, CUDA 12.8 runtime for e2e ONNX inference, ONNXRuntime GPU, NumPy/OpenCV, Hugging Face model assets mounted from `nav_model_zoo/`. PyTorch CUDA wheels are optional for checkpoint/development images, not required by the default VLM or S2E ONNX runtime path.

---

## File Responsibilities

- `src/s2e_vlm_core/s2e_vlm_core/s2e_backend.py`: pure S2E preprocessing, 11-frame context buffer, trajectory output validation/conversion, and optional navigator wrapper.
- `src/s2e_vlm_core/test/test_s2e_backend.py`: unit tests for image conversion, frame buffering, S2E output conversion, and fake navigator behavior.
- `src/s2e_vlm_nodes/s2e_vlm_nodes/ros_mock_runtime.py`: runtime switch for `E2E_BACKEND=mock|s2e`, image context waiting status, S2E planner invocation, and trajectory publication.
- `tests/test_docker_assets.py`: static Docker/Compose/docs contract tests for ONNX runtime dependencies, model mounts, backend env, Qwen API env, and ignored model assets.
- `docker/onnx-runtime-base.Dockerfile`: install CUDA/cuDNN, ROS 2 Jazzy, and ONNXRuntime GPU without PyTorch for the S2E runtime.
- `docker/gpu-inference-base.Dockerfile`: optional heavyweight PyTorch CUDA image for checkpoint experiments or future dev/training flows.
- `compose.yaml` and `.env.example`: document/pass `E2E_BACKEND`, `E2E_MODEL_PATH`, `VLM_BACKEND`, `VLM_API_URL`, timeout/retry settings, and model mount defaults.
- `README.md`, `docs/testing.md`, `docs/requirements.md`: operational docs for Docker-based Hugging Face download, S2E single-PC validation, VLM Qwen API backend, and known limits of white-image semantic testing.

## Tasks

### Task 1: Static Docker and Documentation Contracts

**Files:**
- Modify: `tests/test_docker_assets.py`
- Modify: `.gitignore`

- [ ] Write failing tests requiring `nav_model_zoo/` to be ignored, ONNXRuntime GPU to be installed in `onnx-runtime-base`, e2e S2E env/mounts in Compose, VLM API env examples, and no PyTorch/GPU requirement in the default VLM runtime.
- [ ] Run `python3 -m unittest tests/test_docker_assets.py -v` and confirm the new assertions fail.
- [ ] Implement the minimal Docker/Compose/env/gitignore changes.
- [ ] Re-run the test and confirm it passes.

### Task 2: Pure S2E Preprocessing and Output Conversion

**Files:**
- Create: `src/s2e_vlm_core/s2e_vlm_core/s2e_backend.py`
- Create: `src/s2e_vlm_core/test/test_s2e_backend.py`

- [ ] Write tests for `rgb8` ROS-like image bytes converting to `(3, 256, 256)` float32 `[0, 1]`.
- [ ] Write tests for an 11-frame context buffer returning no batch until full, then `(1, 11, 3, 256, 256)`.
- [ ] Write tests for `(1, 1, 10, 2)` S2E output converting to exactly 10 finite `(x, y)` points.
- [ ] Write tests for invalid S2E output shape/value rejection.
- [ ] Run `python3 -m unittest src/s2e_vlm_core/test/test_s2e_backend.py -v` and confirm RED failures.
- [ ] Implement the pure module.
- [ ] Re-run tests and confirm GREEN.

### Task 3: e2e Runtime Backend Switch

**Files:**
- Modify: `src/s2e_vlm_nodes/s2e_vlm_nodes/ros_mock_runtime.py`
- Modify: `src/s2e_vlm_nodes/test/test_dummy_integration.py` or add targeted node-runtime tests if ROS is required.

- [ ] Write tests or static assertions proving `E2E_BACKEND=s2e` waits for 11 frames before publishing a trajectory.
- [ ] Wire `E2E_BACKEND` and `E2E_MODEL_PATH` into `E2EMockNode` without changing default mock behavior.
- [ ] Add `WAITING_IMAGE_CONTEXT` status while fewer than 11 frames are buffered.
- [ ] Call S2E planner only after image/pose/VLM/supervisor gates pass.
- [ ] Convert S2E points into existing `Trajectory2D` fields.
- [ ] Run existing node/core tests and ensure mock behavior still passes.

### Task 4: S2E Docker Smoke Validation

**Files:**
- Modify: `docs/testing.md`
- Modify: `README.md`

- [ ] Build the GPU image.
- [ ] Run `onnxruntime.get_available_providers()` inside `s2e-e2e` with `--gpus all`.
- [ ] Run mounted `nav_model_zoo/S2E/s2e.onnx` smoke inference and confirm CUDA provider plus finite `(1, 1, 10, 2)` output.
- [ ] Run single-PC S2E backend graph long enough to produce a debug visualizer artifact.
- [ ] Document artifact results and limitations: white dummy images validate integration, not driving quality.

### Task 5: VLM Qwen API Backend Design Surface

**Files:**
- Modify: `.env.example`
- Modify: `compose.yaml`
- Modify: `README.md`
- Modify: `docs/requirements.md`

- [ ] Add runtime configuration for `VLM_BACKEND=mock|qwen_api`, `VLM_API_URL`, `VLM_API_TIMEOUT_S`, and `VLM_API_MAX_RETRIES`.
- [ ] Document that `vlm_node` remains the agentic ROS orchestrator: context collection, prompt/request building, response parsing, strict schema normalization, safety filtering, and status heartbeat.
- [ ] Document that Qwen3-VL 32B Thinking runs as an external model server via API, not inside the ROS node process.
- [ ] Defer full HTTP client implementation to a follow-up task unless explicitly requested in this iteration.

### Task 6: Final Verification

**Files:**
- All touched files.

- [ ] Run `python3 -m unittest tests/test_docker_assets.py -v`.
- [ ] Run `python3 -m unittest src/s2e_vlm_core/test/test_s2e_backend.py -v`.
- [ ] Run `python3 -m unittest src/s2e_vlm_bringup/test_launch_contracts.py -v`.
- [ ] Run `python3 -m compileall src tests`.
- [ ] Run LSP diagnostics on changed Python/Markdown/YAML files where available.
- [ ] Run Docker Compose config validation for the updated profiles.
- [ ] Summarize which parts were fully implemented and which Qwen API serving details remain as documented follow-up.
