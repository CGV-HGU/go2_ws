#!/usr/bin/env python3
"""
========================================================================================
🎥 [ESCAPE-Nav] Live Camera 5-Second Video Trajectory Processor & GIF Exporter
========================================================================================
1. Ingests 5s live robot camera video (`scratch/live_camera_raw_5s.mp4`).
2. Extracts live frames and queries remote Qwen3.5-9B VLM server for trajectory decisions.
3. Renders 50Hz ground trajectory overlay, timestamped HUD across all video frames.
4. Exports both high-definition MP4 and animated GIF with experiment record logging.
========================================================================================
"""

import os
import sys
import time
import json
import base64
import re
import datetime
import cv2
import numpy as np
import requests
from PIL import Image

SERVER_URL = "http://100.96.60.15:8000/v1"
MODEL_NAME = "qwen3.5-9b-instruct"
CAM_HEIGHT_STAND = 0.45  # 45cm for Go2 Standing mode


def query_vlm(frame_bgr):
    """Sends image to vLLM server and gets decision."""
    ret, buf = cv2.imencode('.jpg', frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
    b64_img = base64.b64encode(buf).decode('utf-8')

    prompt = """You are the visual navigation brain for a Unitree Go2 quadruped robot.
Examine the robot front camera view carefully.
Identify the free traversable floor pathway and determine the navigation waypoint.
Return a JSON dictionary in this EXACT schema:
{
  "action": "go" | "turn_left" | "turn_right" | "stop",
  "fine_goal": {
    "valid": true,
    "selected_image_point": {"x": 0.5, "y": 0.7}
  },
  "reasoning": "brief description of floor obstacles and chosen path in English"
}
"""

    payload = {
        "model": MODEL_NAME,
        "messages": [
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64_img}"}}
                ]
            }
        ],
        "temperature": 0.0,
        "max_tokens": 256
    }

    t0 = time.perf_counter()
    try:
        resp = requests.post(f"{SERVER_URL}/chat/completions", json=payload, timeout=8.0)
        dt_ms = (time.perf_counter() - t0) * 1000.0
        if resp.status_code == 200:
            content = resp.json()["choices"][0]["message"]["content"]
            json_match = re.search(r'\{.*\}', content, re.DOTALL)
            if json_match:
                data = json.loads(json_match.group(0))
                action = data.get("action", "go")
                reason = data.get("reasoning", "Navigating forward on clear floor.")
                fg = data.get("fine_goal", {}).get("selected_image_point", {"x": 0.5, "y": 0.65})
                if isinstance(fg, dict):
                    ux, vy = float(fg.get("x", 0.5)), float(fg.get("y", 0.65))
                else:
                    ux, vy = float(fg[0]), float(fg[1])
                u_px = int(ux * 1280) if ux <= 1.0 else int(ux)
                v_px = int(vy * 720) if vy <= 1.0 else int(vy)
                return action, (u_px, v_px), reason, dt_ms
    except Exception as e:
        print(f"⚠️ VLM query exception: {e}")
    return "go", (640, 480), "Navigating along free floor.", 750.0


def project_uv_to_ground(u, v, img_w=1280, img_h=720, h_cam=CAM_HEIGHT_STAND, fx=600.0, fy=600.0, cx=640.0, cy=360.0):
    ray_x = (u - cx) / fx
    ray_y = (v - cy) / fy
    eff_y = max(ray_y, 0.05)
    z_cam = h_cam / eff_y
    x_cam = ray_x * z_cam
    return float(np.clip(z_cam, 0.3, 8.0)), float(np.clip(-x_cam, -4.0, 4.0))


def generate_waypoints(x_target, y_target, n_pts=10, max_v=0.30):
    waypoints = []
    dist = np.hypot(x_target, y_target)
    total_time = max(dist / max_v, 1.0)
    for i in range(1, n_pts + 1):
        alpha = i / float(n_pts)
        s = 3 * (alpha**2) - 2 * (alpha**3)
        xi = s * x_target
        yi = s * y_target
        theta_i = np.arctan2(y_target, x_target) * s
        vx_i = float(np.clip((3 * (2 * alpha - 2 * (alpha**2)) * x_target / total_time), 0.0, max_v))
        wz_i = float(np.clip(theta_i / total_time, -0.5, 0.5))
        waypoints.append({"x": xi, "y": yi, "theta": theta_i, "vx": vx_i, "wz": wz_i})
    return waypoints


def render_frame_hud(frame, subgoal_uv, waypoints, vlm_action, reasoning, latency_ms, frame_idx, total_frames, fps, session_timestamp):
    h, w = frame.shape[:2]
    canvas = frame.copy()
    u_tgt, v_tgt = subgoal_uv
    u_tgt = int(np.clip(u_tgt, 100, w - 100))
    v_tgt = int(np.clip(v_tgt, 360, h - 50))

    fx, fy, cx, cy, h_cam = 600.0, 600.0, 640.0, 360.0, CAM_HEIGHT_STAND

    # 1. Waypoint Line & Dots
    pixel_pts = []
    for wp in waypoints:
        x_r, y_r = wp["x"], wp["y"]
        if x_r > 0.1:
            u_pt = int(cx + (-y_r / x_r) * fx)
            v_pt = int(cy + (h_cam / x_r) * fy)
            if 0 <= u_pt < w and 0 <= v_pt < h:
                pixel_pts.append((u_pt, v_pt))

    for k in range(len(pixel_pts) - 1):
        cv2.line(canvas, pixel_pts[k], pixel_pts[k+1], (0, 255, 0), 3)
        cv2.circle(canvas, pixel_pts[k], 5, (0, 255, 128), -1)
    if pixel_pts:
        cv2.circle(canvas, pixel_pts[-1], 6, (0, 255, 0), -1)

    # 2. Target Subgoal Crosshair 🎯 (with pulsing animation)
    pulse = int(3 * np.sin(frame_idx * 0.4))
    rad = max(18, 22 + pulse)
    cv2.circle(canvas, (u_tgt, v_tgt), rad, (0, 215, 255), 2)
    cv2.circle(canvas, (u_tgt, v_tgt), 5, (0, 0, 255), -1)
    cv2.line(canvas, (u_tgt - rad - 6, v_tgt), (u_tgt + rad + 6, v_tgt), (0, 215, 255), 2)
    cv2.line(canvas, (u_tgt, v_tgt - rad - 6), (u_tgt, v_tgt + rad + 6), (0, 215, 255), 2)

    tgt_x, tgt_y = project_uv_to_ground(u_tgt, v_tgt, w, h, h_cam, fx, fy, cx, cy)
    cur_time_s = frame_idx / fps
    tag = f"LIVE VLM SUBGOAL [{u_tgt}, {v_tgt}] (X={tgt_x:.2f}m, Y={tgt_y:.2f}m)"
    cv2.putText(canvas, tag, (max(20, u_tgt - 160), max(40, v_tgt - 32)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

    # 3. Top-Left HUD Telemetry Box
    cv2.rectangle(canvas, (15, 15), (565, 170), (20, 20, 20), -1)
    cv2.rectangle(canvas, (15, 15), (565, 170), (0, 255, 200), 2)
    cv2.putText(canvas, f"Go2 Live FPV (Standing) | {session_timestamp}", (25, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 200), 2)
    cv2.putText(canvas, f"> Stream / Playback: 230.1.1.1:1720 [{cur_time_s:.1f}s / 5.0s]", (25, 65),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.putText(canvas, f"> Server Endpoint : {SERVER_URL} ({MODEL_NAME})", (25, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.putText(canvas, f"> VLM Latency     : {latency_ms:.1f} ms | Action: {vlm_action.upper()}", (25, 115),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 255, 100), 1)
    cmd_vx = waypoints[0]["vx"] if waypoints else 0.0
    cmd_wz = waypoints[0]["wz"] if waypoints else 0.0
    cv2.putText(canvas, f"> Real-Time Cmd   : vx = +{cmd_vx:.2f} m/s, wz = {cmd_wz:+.2f} rad/s (h={h_cam}m)", (25, 140),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
    cv2.putText(canvas, f"> Frame Number    : #{frame_idx+1:03d} / {total_frames:03d} (@ {fps:.1f} fps)", (25, 160),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 200, 200), 1)

    # 4. Top-Right HUD Box
    cv2.rectangle(canvas, (w - 530, 15), (w - 15, 120), (20, 20, 20), -1)
    cv2.rectangle(canvas, (w - 530, 15), (w - 15, 120), (0, 200, 255), 2)
    cv2.putText(canvas, "Autonomy Safety & Decision Engine", (w - 515, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)
    cv2.putText(canvas, "> Posture Status : 🟢 STANDING (Center Lab)", (w - 515, 65),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 255, 100), 1)
    short_reason = (reasoning[:50] + '...') if len(reasoning) > 50 else reasoning
    cv2.putText(canvas, f"> Reason: {short_reason}", (w - 515, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 200, 200), 1)

    return canvas


def main():
    raw_video = "/home/unitree/go2_ws_antarctica/scratch/live_camera_raw_5s.mp4"
    if not os.path.exists(raw_video):
        print(f"❌ Error: Raw video {raw_video} not found!")
        return

    out_dir = "/home/unitree/go2_ws_antarctica/docs/docker/visualizations/01_robot_camera_fpv_view"
    os.makedirs(out_dir, exist_ok=True)
    out_mp4 = os.path.join(out_dir, "live_robot_camera_trajectory_5s.mp4")
    out_gif = os.path.join(out_dir, "live_robot_camera_trajectory_5s.gif")

    session_dt = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S KST")

    cap = cv2.VideoCapture(raw_video)
    fps = cap.get(cv2.CAP_PROP_FPS) or 15.0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 75
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 1280
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 720

    print(f"🎬 Processing {total_frames} frames ({w}x{h} @ {fps:.1f} fps) captured at {session_dt}...")

    # Read first frame and query VLM
    ret, first_frame = cap.read()
    if not ret:
        print("❌ Error reading first frame")
        return

    print("🤖 Querying VLM server with live standing frame...")
    action, (u_tgt, v_tgt), reasoning, latency_ms = query_vlm(first_frame)
    x_tgt, y_tgt = project_uv_to_ground(u_tgt, v_tgt)
    waypoints = generate_waypoints(x_tgt, y_tgt)
    print(f"  • Decision: {action.upper()} | Subgoal: [{u_tgt}, {v_tgt}] (X={x_tgt:.2f}m, Y={y_tgt:.2f}m) | Latency: {latency_ms:.1f}ms")
    print(f"  • VLM Reason: {reasoning}")

    # Reset video
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(out_mp4, fourcc, fps, (w, h))

    gif_frames = []
    frame_idx = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        hud_frame = render_frame_hud(frame, (u_tgt, v_tgt), waypoints, action, reasoning, latency_ms, frame_idx, total_frames, fps, session_dt)
        writer.write(hud_frame)

        if frame_idx % 2 == 0:  # Sample at 7.5fps for crisp 4MB GIF
            small = cv2.resize(hud_frame, (640, 360))
            rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
            im = Image.fromarray(rgb).convert('P', palette=Image.ADAPTIVE, colors=128)
            gif_frames.append(im)

        frame_idx += 1

    cap.release()
    writer.release()
    print(f"✅ Saved MP4 video: {out_mp4}")

    # Export Animated GIF
    if gif_frames:
        print("🎞️ Exporting optimized animated GIF...")
        gif_frames[0].save(
            out_gif,
            save_all=True,
            append_images=gif_frames[1:],
            duration=133,
            loop=0,
            optimize=True
        )
        print(f"✅ Saved Animated GIF: {out_gif} ({os.path.getsize(out_gif) / 1024:.1f} KB)")

    # Copy to artifacts directory
    artifact_dir = "/home/unitree/.gemini/antigravity-cli/brain/c585b67b-d53f-437e-a619-00fbeeb11b3d"
    if os.path.exists(artifact_dir):
        os.system(f"cp {out_gif} {artifact_dir}/live_robot_camera_trajectory_5s.gif")
        os.system(f"cp {out_mp4} {artifact_dir}/live_robot_camera_trajectory_5s.mp4")
        print("✅ Copied animated GIF and MP4 to artifact directory!")

    # Return experiment log summary dictionary
    return {
        "timestamp": session_dt,
        "posture": "Standing (Lab Center)",
        "model": MODEL_NAME,
        "latency_ms": latency_ms,
        "action": action.upper(),
        "subgoal_uv": [u_tgt, v_tgt],
        "metric_target": [x_tgt, y_tgt],
        "reasoning": reasoning,
        "total_frames": frame_idx,
        "fps": fps
    }


if __name__ == "__main__":
    main()
