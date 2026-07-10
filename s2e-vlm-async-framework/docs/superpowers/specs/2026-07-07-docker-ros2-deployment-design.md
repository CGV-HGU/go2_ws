# Docker ROS 2 Deployment Design

## Purpose

This design records how the project will use Docker and Docker Compose for the S2E VLM async framework. The goal is to isolate fragile GPU model runtimes without turning every ROS 2 node into a separate container. The first implementation target remains single-PC development and mock validation. The Go2 robot plus external GPU PC split is the target deployment path and must be supported by the same image and Compose structure.

## Decision Summary

Use Docker images grouped by dependency and deployment boundary, not one image per ROS 2 node.

The preferred service groups are:

- `robot-core`: robot-side ROS 2 nodes such as sensors, odometry, controller, supervisor, and optional debug visualizer. This image is CPU-only and has no CUDA, cuDNN, or TensorRT dependency.
- `vlm`: VLM ROS 2 node and its GPU model runtime. This image requests GPU access and inherits a pinned GPU inference base with CUDA, cuDNN, TensorRT, ONNX, and ONNX Runtime GPU support.
- `e2e`: e2e ROS 2 node and its GPU model runtime. This image requests GPU access and inherits the same pinned GPU inference base, then adds e2e-specific TensorRT engines, plugins, and adapters.
- `dev-mock`: optional single-PC development image for fast mock validation and local testing before hardware integration.

ROS 2 launch files remain the source of truth for which nodes run together. Docker Compose only selects the runtime environment, machine boundary, volumes, GPU access, and network settings.

## Considered Approaches

### Approach A: One All-In-One Image

An all-in-one image runs every node and every model runtime in one container. This is useful for early smoke tests because it minimizes ROS 2 DDS and Compose complexity. It is not the production design because it puts CUDA, cuDNN, TensorRT, and large model dependencies into the same environment as controller and odometry code.

Use this only as an optional `dev-mock` profile.

### Approach B: Grouped Images by Runtime Boundary

Grouped images split services by dependency profile and deployment boundary. Robot-side nodes run in a lightweight CPU-only image. VLM and e2e run in separate GPU-capable images so their model dependencies can evolve independently.

This is the selected approach because it matches the architecture split, keeps controller and odometry simple, and avoids premature per-node container overhead.

### Approach C: One Image or Container per ROS 2 Node

Per-node images give maximum isolation, but they increase DDS discovery surface area, duplicate dependencies, complicate launch orchestration, and force more inter-container serialization. This project does not need that complexity in the first implementation.

Avoid this approach unless a future node has a hard dependency conflict, independent release cadence, or crash-isolation requirement that cannot be handled by grouped services.

## Deployment Modes

### Mode 1: Single-PC Mock Development

All mock nodes run on one development PC. This is the first implementation and test target.

Expected use:

- Validate ROS 2 packages, messages, actions, launch files, state transitions, TTL handling, stale-cache behavior, and rotate preemption.
- Run without Go2 hardware.
- Prefer a simple `dev-mock` container or native host execution while package structure is still changing.

### Mode 2: Single-PC Split Containers

Robot-side, VLM, and e2e services run as separate containers on the same host. This validates that ROS 2 communication works across container boundaries before introducing two physical machines.

Expected use:

- `robot-core` container runs mock sensor, mock odometry, controller, and supervisor nodes.
- Debug visualizer may run in `robot-core` or `dev-mock` when overlay output is useful.
- `vlm` container runs the VLM node with GPU access if a model is available.
- `e2e` container runs the e2e node with GPU access if a model is available.
- Host networking and host IPC are used first to reduce DDS discovery surprises.

### Mode 3: Go2 Robot plus External GPU PC

Robot-side services run on the Go2 computer or a robot-attached compute device. VLM and e2e services run on an external GPU PC. This is the target deployment path after single-PC validation passes.

Expected use:

- Robot side owns sensor acquisition, odometry, controller, rotate action server, heartbeat checks, and fail-closed behavior.
- Debug visualizer can run on the robot side, external GPU PC, or a development host as a CPU-only observer, depending on where image bandwidth and display access are most convenient.
- External GPU PC owns VLM reasoning and e2e trajectory inference.
- Both machines use the same ROS 2 interface definitions and compatible RMW configuration.

## Compose and ROS 2 Networking Requirements

Use host networking for the initial Compose profiles:

```yaml
network_mode: host
ipc: host
```

Host networking and host IPC are development/lab defaults, not security controls. Before real robot commands are enabled, deployment must use an isolated robot network or firewall allowlist for DDS peers. `ROS_DOMAIN_ID` only separates ROS graphs by convention and must not be treated as authentication.

Every host and container participating in the same ROS 2 graph must share:

```bash
ROS_DOMAIN_ID=<same-domain-id>
RMW_IMPLEMENTATION=<same-rmw>
ROS_LOCALHOST_ONLY=0
```

Use a non-default `ROS_DOMAIN_ID` for lab and robot tests to avoid accidental cross-talk with other ROS 2 systems. The same domain ID is necessary but not sufficient: firewalls, VPNs, multicast routing, Docker networking, and NIC selection can still block DDS discovery.

Pin one RMW implementation across all images. The current docs mention `rmw_fastrtps_cpp`; if multi-machine discovery is unreliable, add explicit Fast DDS or Cyclone DDS interface configuration rather than mixing RMWs. For real robot deployments, add DDS Security/SROS2 or an equivalent authenticated transport for `/s2e/e2e/trajectory`, `/s2e/vlm/reasoning`, all enumerated `/s2e/status/<node_name>` topics, `/s2e/supervisor/health`, and `/s2e/controller/rotate`.

## GPU Runtime Requirements

Only VLM and e2e services request GPU devices. Robot-side containers must not depend on NVIDIA runtime packages.

Both GPU model services should share a common GPU inference base image instead of independently installing ad hoc model runtimes. This base should pin:

- CUDA userspace libraries compatible with the host NVIDIA driver.
- cuDNN for neural network runtime kernels.
- TensorRT runtime and ONNX parser support for optimized inference and engine loading/building.
- ONNX Python tooling for model inspection/conversion workflows.
- ONNX Runtime GPU, with CUDA and TensorRT execution providers if the selected versions are compatible.

This is intentional dependency symmetry for the model side, not a reason to put GPU dependencies in `robot-core`. The cost is larger VLM and e2e images, but the benefit is fewer rebuilds when a model moves between PyTorch-exported ONNX, ONNX Runtime, TensorRT, or TensorRT-backed execution.

GPU hosts need NVIDIA Container Toolkit configured on the host. Compose services should reserve explicit GPU devices where possible, for example:

```yaml
deploy:
  resources:
    reservations:
      devices:
        - driver: nvidia
          device_ids: ["0"]
          capabilities: [gpu]
```

Use `NVIDIA_DRIVER_CAPABILITIES=compute,utility` for inference-only services unless a service explicitly needs graphics, video, or display capabilities.

The reference e2e runtime expects a CUDA/TensorRT-specific environment. Treat TensorRT engines as hardware/runtime-specific artifacts and mount model or engine directories into the container instead of baking changing model files into the base image. If VLM later uses a runtime that does not need TensorRT, keep the shared base for development reproducibility and consider a slim VLM production image only after the runtime is stable.

## Image Structure

The implementation should build images in this order:

1. `s2e-ros-base`: ROS 2, shared project packages, generated messages/actions, common launch/config utilities.
2. `s2e-dev-mock`: extends `s2e-ros-base`; includes test/mock dependencies and can launch the single-PC mock stack.
3. `s2e-robot`: extends `s2e-ros-base`; includes robot-side runtime dependencies only.
4. `s2e-gpu-inference-base`: compatible ROS/GPU base with CUDA, cuDNN, TensorRT, ONNX tooling, and ONNX Runtime GPU support pinned as one compatibility unit.
5. `s2e-vlm`: extends `s2e-gpu-inference-base`; includes VLM runtime dependencies and model adapter code.
6. `s2e-e2e`: extends `s2e-gpu-inference-base`; includes e2e-specific runtime dependencies, TensorRT plugins or engines, and model adapter code.

If ROS 2 distro and GPU base OS versions conflict, prefer a documented compatibility matrix over silently mixing unsupported binaries. The e2e reference is Ubuntu 22.04, Python 3.10, CUDA 11.8, cuDNN8, and TensorRT 8.5.1.7. ONNX Runtime GPU must be pinned to a build compatible with the chosen CUDA, cuDNN, and TensorRT versions. The ROS 2 implementation must pin a compatible distro before Dockerfiles are finalized.

## Compose Profiles

The project should provide one `compose.yaml` with profiles rather than many unrelated compose files:

- `single_pc_mock`: runs `dev-mock` or equivalent mock stack on one PC.
- `single_pc_split`: runs `robot-core`, optional debug visualizer, `vlm`, and `e2e` as separate services on one PC.
- `robot_side`: runs only robot-side services on the Go2 side.
- `external_gpu`: runs VLM and e2e services on the external GPU PC, with optional debug visualizer if that host should publish the annotated image.

The same environment variable names should be used across profiles. A `.env.example` should document `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, `ROS_LOCALHOST_ONLY`, model mount paths, and GPU device selection.

## Safety and Failure Handling

Container boundaries must not change safety behavior.

- Controller remains the only robot motion authority.
- Loss of VLM, e2e, odometry, trajectory, or heartbeat still degrades to stop or hold.
- Robot-side supervisor publishes `/s2e/supervisor/health`; controller treats `ok_to_move=false` as motion-blocking.
- Rotate actions still preempt trajectory following and return control only on success, cancel, timeout, or fault.
- Malformed VLM strings and stale cached reasoning are rejected regardless of whether the producer is local or remote.

The robot-side stack must remain capable of stopping safely when external GPU services disappear.

## Testing Strategy

Testing proceeds in increasing deployment complexity:

1. Native or `dev-mock` single-PC mock launch.
2. Single-PC split containers with host networking.
3. Single-PC split containers with GPU model services enabled.
4. Two-machine robot/external-PC smoke test.
5. Hardware-in-the-loop Go2 test with controller fail-closed checks.

Each mode should verify:

- `ros2 topic list` shows expected topics.
- `ros2 action list` shows `/s2e/controller/rotate`.
- `/s2e/e2e/trajectory` publishes with valid stamps and `base_link` frame semantics.
- `/s2e/debug/visualizer/image` publishes when the debug visualizer profile is enabled.
- Heartbeats are visible at the expected rate.
- Killing VLM or e2e containers causes safe degradation, not stale motion.
- Restarting GPU services recovers without changing topic contracts.

Track latency and throughput separately for native, single-container, same-host split, and two-machine split modes. Container overhead is expected to be less important than DDS serialization, image transport, GPU contention, and model inference time.

## Non-Goals

- Do not introduce Kubernetes or an orchestrator beyond Docker Compose.
- Do not create one image per ROS 2 node in the initial implementation.
- Do not require CUDA or TensorRT on the robot-side controller and odometry image.
- Do not require GPU access for the debug visualizer; it is an OpenCV/cv_bridge CPU observer unless a future implementation explicitly documents acceleration needs.
- Do not install GPU inference libraries separately in each model image when a shared pinned GPU base can provide the same compatibility contract.
- Do not use `network_mode: host`, `ipc: host`, or `ROS_DOMAIN_ID` as a security boundary for real robot deployments.
- Do not copy proprietary reference process managers directly; use ROS 2 launch, lifecycle behavior, diagnostics, and project-specific supervision.

## Open Implementation Notes

Before writing Dockerfiles, pin the ROS 2 distro and base OS combination against the GPU runtime constraints. If the first implementation uses mock-only nodes, Dockerfiles may start with CPU-only ROS images and add GPU images after the package interfaces stabilize.
