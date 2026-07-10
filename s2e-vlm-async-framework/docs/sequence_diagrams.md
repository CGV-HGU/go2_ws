# Sequence Diagrams

These diagrams use Mermaid syntax and define required behavior for implementation and tests.

## Normal Async Sensor to Control Flow

```mermaid
sequenceDiagram
    participant L as lidar_node
    participant C as camera_node
    participant I as imu_node
    participant O as odometry_node
    participant V as vlm_node
    participant E as e2e_node
    participant R as controller_node
    participant W as supervisor_node
    participant D as debug_visualizer_node

    par Independent sensor publishing
        L->>O: /s2e/sensors/lidar/points<br/>header.stamp=t_lidar, frame_id=lidar
    and
        C->>V: /s2e/sensors/camera/image<br/>header.stamp=t_img, frame_id=camera
        C->>E: /s2e/sensors/camera/image<br/>header.stamp=t_img, frame_id=camera
        C->>D: /s2e/sensors/camera/image<br/>raw image for overlay
    and
        I->>O: /s2e/sensors/imu<br/>header.stamp=t_imu, frame_id=imu
    end

    O->>V: /s2e/odometry/pose<br/>pose(base_link in odom), stamp=t_pose
    O->>E: /s2e/odometry/pose<br/>pose(base_link in odom), stamp=t_pose
    O->>R: /s2e/odometry/pose<br/>faster control-rate pose stream

    V->>V: lookup latest pose <= t_img within 0.20s
    V->>V: run VLM at slow cadence
    V->>E: /s2e/vlm/reasoning<br/>strict JSON string at t_img
    V->>D: /s2e/vlm/reasoning<br/>action, reasoning, coarse goal_uv

    E->>E: lookup latest pose <= image stamp within 0.20s
    E->>E: parse cached VLM JSON
    E->>E: image uv goal -> base_link 2D goal
    E->>E: compensate goal from VLM pose to current pose
    E->>E: run e2e model
    E->>R: /s2e/e2e/trajectory<br/>10x2 base_link points + pose_at_trajectory
    E->>D: /s2e/e2e/trajectory<br/>trajectory + fine goal point
    W->>R: /s2e/supervisor/health<br/>ok_to_move=true

    R->>R: transform trajectory from pose_at_trajectory to current pose
    R->>R: PID trajectory following
    R->>D: /s2e/controller/status<br/>mode and rotate debug fields
    D->>D: draw image overlay and mini-map
    D->>D: publish /s2e/debug/visualizer/image
    R->>R: publish robot command or mock Twist
```

## E2E Reusing Cached VLM Reasoning

```mermaid
sequenceDiagram
    participant V as vlm_node
    participant E as e2e_node
    participant O as odometry_node
    participant R as controller_node

    V->>E: VLM JSON(action=go, goal_uv, pose_at_vlm, stamp=t_vlm)
    loop Faster e2e cycles
        O->>E: pose_t_current
        E->>E: check VLM TTL <= 8.0s
        E->>E: compute relative transform pose_at_vlm -> pose_t_current
        alt Transform within bounds
            E->>E: adjust goal point
            E->>R: publish new trajectory with current image stamp
        else Transform too large or stale
            E->>E: enter DEGRADED
            E->>R: publish status and no new trajectory
        end
    end
```

## Rotate Action Flow

```mermaid
sequenceDiagram
    participant V as vlm_node
    participant R as controller_node
    participant O as odometry_node
    participant D as debug_visualizer_node

    V->>V: VLM output JSON(action=rotate, rotate_deg=x)
    V->>R: Rotate action goal(target_yaw_delta_deg=x)
    R->>O: read latest pose
    alt Odometry fresh and controller healthy
        R-->>V: goal accepted
        R->>R: clear trajectory and enter ROTATING
        R->>R: record start yaw
        V->>V: enter FROZEN_ROTATING
        loop Rotate until target reached
            O->>R: current pose
            R->>R: compute yaw delta and remaining angle
            R-->>V: action feedback(current_yaw_delta, remaining)
            R->>D: controller status(current_yaw_delta, remaining)
            R->>R: publish yaw command
        end
        R->>R: hold tolerance for settle time
        R-->>V: action result(success=true)
        R->>D: controller status(last rotate result success)
        V->>V: exit FROZEN_ROTATING
    else Odometry stale or fault
        R-->>V: goal rejected or aborted
        V->>V: exit freeze and publish degraded status
        R->>R: stop/hold command
    end
```

## Debug Visualizer Overlay Flow

```mermaid
sequenceDiagram
    participant C as camera_node
    participant V as vlm_node
    participant E as e2e_node
    participant R as controller_node
    participant D as debug_visualizer_node
    participant H as human_debug_viewer

    C->>D: camera image with stamp=t_img
    V->>D: latest VLM JSON action, reasoning, goal_uv
    E->>D: latest trajectory and goal_point_base_link
    E->>D: e2e status cache age and validity
    R->>D: controller status mode, rotate remaining, result
    D->>D: cache latest debug state by source stamp
    D->>D: draw goal_uv on image
    D->>D: draw base_link trajectory in 2D mini-map
    alt CameraInfo and transforms available
        D->>D: project base_link points into camera image
    else Projection unavailable
        D->>D: label projection unavailable and keep mini-map
    end
    D->>H: /s2e/debug/visualizer/image annotated image
```

## Stop Command Flow

```mermaid
sequenceDiagram
    participant V as vlm_node
    participant E as e2e_node
    participant R as controller_node

    V->>E: VLM JSON(action=stop)
    E->>E: enter STOPPED_BY_VLM
    E->>R: /s2e/e2e/status STOPPED_BY_VLM
    R->>R: clear or ignore stale trajectory if stop is active
    R->>R: command zero velocity / hold
```

## Malformed VLM String

```mermaid
sequenceDiagram
    participant V as vlm_node
    participant E as e2e_node
    participant R as controller_node

    V->>E: /s2e/vlm/reasoning invalid JSON
    E->>E: parser rejects payload
    E->>E: mark reasoning invalid and do not infer trajectory
    E->>R: publish DEGRADED status
    R->>R: if no valid trajectory remains, stop/hold
```

## Watchdog Failure

```mermaid
sequenceDiagram
    participant N as any_node
    participant S as supervisor_node
    participant R as controller_node

    loop Healthy
        N->>S: /s2e/status/node_name heartbeat at 1 Hz
    end
    N-xS: heartbeat missing for 3 periods
    S->>S: mark node unhealthy
    alt Failed node affects controller safety
        S->>R: /s2e/supervisor/health ok_to_move=false
        R->>R: stop/hold
    else Non-critical external node failed
        S->>S: restart or report degraded mode
    end
```

## Robot/PC Split

```mermaid
sequenceDiagram
    box Robot Computer
        participant S as sensor nodes
        participant O as odometry_node
        participant R as controller_node
        participant W as supervisor_node
        participant D as optional debug_visualizer_node
    end
    box External PC
        participant V as vlm_node
        participant E as e2e_node
    end

    S->>O: local sensor topics
    O->>V: pose stream over ROS 2 network
    O->>E: pose stream over ROS 2 network
    S->>V: camera image over ROS 2 network
    S->>E: camera image over ROS 2 network
    V->>E: VLM reasoning string
    V->>D: VLM reasoning string for overlay
    E->>R: trajectory + pose_at_trajectory over ROS 2 network
    E->>D: trajectory + fine goal for overlay
    V->>W: /s2e/status/vlm_node heartbeat over ROS 2 network
    E->>W: /s2e/status/e2e_node heartbeat over ROS 2 network
    V->>R: Rotate action goal/result over ROS 2 network
    R->>D: controller status and rotate progress
    D->>D: publish local annotated debug image when enabled
    W->>R: /s2e/supervisor/health ok_to_move=false if external PC heartbeats disappear
    R->>R: fail closed on degraded health or trajectory TTL
```
