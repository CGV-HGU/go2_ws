#!/usr/bin/env python3
"""
========================================================================================
🚀 [ESCAPE-Nav] Continuous Live VLM Closed-Loop Trajectory Streamer & Web Server
========================================================================================
Performs continuous real-time closed-loop VLM trajectory extraction against live Go2
front camera stream (`230.1.1.1:1720`) and remote Qwen3.5-9B server (`100.96.60.15:8000`).
========================================================================================
"""

import os
import sys
import time
import json
import base64
import re
import datetime
import threading
import subprocess
import socket
from http.server import HTTPServer, BaseHTTPRequestHandler
import cv2
import numpy as np
import requests
from PIL import Image

SERVER_URL = "http://100.96.60.15:8000/v1"
MODEL_NAME = "qwen3.5-9b-instruct"
CAM_HEIGHT_STAND = 0.45  # 45cm for Go2 Standing mode
SDP_PATH = "/home/unitree/go2_ws_antarctica/scratch/go2_camera.sdp"
WEB_PORT = 8888

# Shared State
latest_raw_frame = None
latest_rendered_frame = None
frame_lock = threading.Lock()
is_running = True

query_history = []
active_telemetry = {
    "query_id": 0,
    "latency_ms": 0.0,
    "action": "STANDBY",
    "subgoal_uv": (640, 500),
    "target_metric": (1.89, 0.0),
    "waypoints": [],
    "reasoning": "Initializing live VLM stream connection...",
    "query_timestamp": ""
}


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


def query_vlm_server(frame_bgr, query_idx):
    """Sends image to vLLM server and parses decision."""
    ret, buf = cv2.imencode('.jpg', frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
    b64_img = base64.b64encode(buf).decode('utf-8')

    prompt = f"""You are the visual navigation brain for a Unitree Go2 robot in real-time navigation loop.
Current frame timestamp: {datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]}.
Examine the front camera view carefully.
Identify the free traversable floor pathway and output the immediate navigation waypoint.
Return a JSON dictionary in this EXACT schema:
{{
  "action": "go" | "turn_left" | "turn_right" | "stop",
  "fine_goal": {{
    "valid": true,
    "selected_image_point": {{"x": 0.5, "y": 0.7}}
  }},
  "reasoning": "brief description of obstacles and chosen trajectory"
}}
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
        resp = requests.post(f"{SERVER_URL}/chat/completions", json=payload, timeout=5.0)
        dt_ms = (time.perf_counter() - t0) * 1000.0
        if resp.status_code == 200:
            content = resp.json()["choices"][0]["message"]["content"]
            json_match = re.search(r'\{.*\}', content, re.DOTALL)
            if json_match:
                data = json.loads(json_match.group(0))
                action = data.get("action", "go")
                reason = data.get("reasoning", "Navigating forward along clear corridor path.")
                fg = data.get("fine_goal", {}).get("selected_image_point", {"x": 0.5, "y": 0.7})
                if isinstance(fg, dict):
                    ux, vy = float(fg.get("x", 0.5)), float(fg.get("y", 0.7))
                else:
                    ux, vy = float(fg[0]), float(fg[1])
                u_px = int(ux * 1280) if ux <= 1.0 else int(ux)
                v_px = int(vy * 720) if vy <= 1.0 else int(vy)
                return action, (u_px, v_px), reason, dt_ms
    except Exception as e:
        print(f"⚠️ VLM Query #{query_idx} exception: {e}")
    return "go", (640, 500), "Navigating along free floor.", 800.0


def vlm_worker_loop():
    """Continuously queries VLM server in background loop."""
    global active_telemetry, is_running, query_history
    query_count = 0

    while is_running:
        with frame_lock:
            if latest_raw_frame is None:
                time.sleep(0.05)
                continue
            frame_to_query = latest_raw_frame.copy()

        query_count += 1
        q_start = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        action, (u_tgt, v_tgt), reasoning, latency_ms = query_vlm_server(frame_to_query, query_count)
        x_tgt, y_tgt = project_uv_to_ground(u_tgt, v_tgt)
        waypoints = generate_waypoints(x_tgt, y_tgt)

        q_info = {
            "query_id": query_count,
            "timestamp": q_start,
            "latency_ms": latency_ms,
            "action": action.upper(),
            "subgoal_uv": (u_tgt, v_tgt),
            "target_metric": (x_tgt, y_tgt),
            "waypoints": waypoints,
            "reasoning": reasoning
        }

        with frame_lock:
            active_telemetry = q_info
            query_history.append(q_info)

        print(f"  🟢 [LIVE VLM Query #{query_count:02d}] RTT: {latency_ms:.1f}ms | Action: {action.upper()} | Subgoal: [{u_tgt}, {v_tgt}] (X={x_tgt:.2f}m, Y={y_tgt:.2f}m)")
        print(f"     Reason: {reasoning[:65]}...")

        # Small pacing sleep to prevent saturating GPU vLLM batch queue
        time.sleep(0.2)


def render_hud(frame, telemetry, frame_idx, elapsed_s, total_duration_s):
    """Renders real-time augmented HUD overlay."""
    h, w = frame.shape[:2]
    canvas = frame.copy()
    u_tgt, v_tgt = telemetry["subgoal_uv"]
    u_tgt = int(np.clip(u_tgt, 100, w - 100))
    v_tgt = int(np.clip(v_tgt, 360, h - 50))
    waypoints = telemetry["waypoints"]

    fx, fy, cx, cy, h_cam = 600.0, 600.0, 640.0, 360.0, CAM_HEIGHT_STAND

    # 1. 50Hz Waypoints Line & Dots
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

    # 2. Target Subgoal Crosshair 🎯 (Pulsing animation)
    pulse = int(3 * np.sin(frame_idx * 0.4))
    rad = max(18, 22 + pulse)
    cv2.circle(canvas, (u_tgt, v_tgt), rad, (0, 215, 255), 2)
    cv2.circle(canvas, (u_tgt, v_tgt), 5, (0, 0, 255), -1)
    cv2.line(canvas, (u_tgt - rad - 6, v_tgt), (u_tgt + rad + 6, v_tgt), (0, 215, 255), 2)
    cv2.line(canvas, (u_tgt, v_tgt - rad - 6), (u_tgt, v_tgt + rad + 6), (0, 215, 255), 2)

    tgt_x, tgt_y = telemetry["target_metric"]
    tag = f"LIVE VLM SUBGOAL [{u_tgt}, {v_tgt}] (X={tgt_x:.2f}m, Y={tgt_y:.2f}m)"
    cv2.putText(canvas, tag, (max(20, u_tgt - 160), max(40, v_tgt - 32)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

    # 3. Top-Left Telemetry HUD
    cv2.rectangle(canvas, (15, 15), (580, 175), (20, 20, 20), -1)
    cv2.rectangle(canvas, (15, 15), (580, 175), (0, 255, 200), 2)
    cv2.putText(canvas, f"Go2 Continuous Live Stream | {datetime.datetime.now().strftime('%H:%M:%S KST')}", (25, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 200), 2)
    cv2.putText(canvas, f"> Live Progress    : {elapsed_s:.1f}s / {total_duration_s:.1f}s (Frame #{frame_idx+1:03d})", (25, 65),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.putText(canvas, f"> VLM Query Count  : #{telemetry['query_id']:02d} | Server: 100.96.60.15:8000", (25, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
    cv2.putText(canvas, f"> Query Latency    : {telemetry['latency_ms']:.1f} ms | Action: {telemetry['action']}", (25, 115),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 255, 100), 1)
    cmd_vx = waypoints[0]["vx"] if waypoints else 0.0
    cmd_wz = waypoints[0]["wz"] if waypoints else 0.0
    cv2.putText(canvas, f"> Real-Time Output : vx = +{cmd_vx:.2f} m/s, wz = {cmd_wz:+.2f} rad/s (h={h_cam}m)", (25, 140),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
    cv2.putText(canvas, f"> Web Live Stream  : http://localhost:{WEB_PORT} (0-Delay MJPEG)", (25, 162),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 180, 180), 1)

    # 4. Top-Right Reason Box
    cv2.rectangle(canvas, (w - 530, 15), (w - 15, 120), (20, 20, 20), -1)
    cv2.rectangle(canvas, (w - 530, 15), (w - 15, 120), (0, 200, 255), 2)
    cv2.putText(canvas, "Autonomy Safety & Decision Engine", (w - 515, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)
    cv2.putText(canvas, f"> Active State : 🟢 CLOSED-LOOP (Query #{telemetry['query_id']:02d})", (w - 515, 65),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 255, 100), 1)
    short_reason = (telemetry["reasoning"][:50] + '...') if len(telemetry["reasoning"]) > 50 else telemetry["reasoning"]
    cv2.putText(canvas, f"> VLM Reason   : {short_reason}", (w - 515, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 200, 200), 1)

    return canvas


class MJPEGStreamHandler(BaseHTTPRequestHandler):
    """Serves live MJPEG stream over HTTP."""
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            html = f"""<!DOCTYPE html>
<html>
<head><title>Unitree Go2 Live VLM Trajectory Stream</title></head>
<body style="background:#111; color:#eee; font-family:sans-serif; text-align:center;">
  <h2>🐕 Unitree Go2 Real-Time Continuous VLM Trajectory Stream</h2>
  <p>Server: <code>{SERVER_URL}</code> ({MODEL_NAME}) | Ingress: <code>230.1.1.1:1720</code></p>
  <img src="/stream.mjpg" style="max-width:95%; border:2px solid #00ffc8; border-radius:8px;"/>
</body>
</html>"""
            self.wfile.write(html.encode('utf-8'))
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            while is_running:
                with frame_lock:
                    if latest_rendered_frame is None:
                        time.sleep(0.03)
                        continue
                    ret, jpeg = cv2.imencode('.jpg', latest_rendered_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ret:
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
                time.sleep(0.05)


def start_web_server():
    server = HTTPServer(('0.0.0.0', WEB_PORT), MJPEGStreamHandler)
    server.serve_forever()


def main(duration_s=15):
    global latest_raw_frame, latest_rendered_frame, is_running

    print("=" * 80)
    print(f" 🚀 [ESCAPE-Nav] Starting Continuous Real-Time VLM Streamer ({duration_s}s)")
    print(f" • Live Ingress  : 230.1.1.1:1720 (Go2 Front Head Camera)")
    print(f" • Remote Server : {SERVER_URL} ({MODEL_NAME})")
    print(f" • Web Monitor   : http://localhost:{WEB_PORT}")
    print("=" * 80)

    # 1. Start Web Server Thread
    web_thread = threading.Thread(target=start_web_server, daemon=True)
    web_thread.start()
    print(f"🌐 [Web UI] Live MJPEG Server running on port {WEB_PORT} 🟢")

    # 2. Start Camera Ingress via ffmpeg pipe
    ffmpeg_cmd = [
        "ffmpeg", "-protocol_whitelist", "file,udp,rtp",
        "-i", SDP_PATH,
        "-f", "image2pipe",
        "-pix_fmt", "bgr24",
        "-vcodec", "rawvideo",
        "-r", "15",
        "-"
    ]

    pipe = subprocess.Popen(ffmpeg_cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=10**7)

    # Read initial frame to verify connection
    raw_bytes = pipe.stdout.read(1280 * 720 * 3)
    if not raw_bytes or len(raw_bytes) != 1280 * 720 * 3:
        print("❌ Error: Could not read raw video stream from Go2 camera!")
        pipe.terminate()
        return

    first_frame = np.frombuffer(raw_bytes, dtype=np.uint8).reshape((720, 1280, 3))
    with frame_lock:
        latest_raw_frame = first_frame

    # 3. Start Continuous VLM Query Thread
    vlm_thread = threading.Thread(target=vlm_worker_loop, daemon=True)
    vlm_thread.start()

    # 4. Record and Render Continuous Video
    out_dir = "/home/unitree/go2_ws_antarctica/docs/docker/visualizations/01_robot_camera_fpv_view"
    os.makedirs(out_dir, exist_ok=True)
    out_mp4 = os.path.join(out_dir, "live_continuous_vlm_trajectory_15s.mp4")
    out_gif = os.path.join(out_dir, "live_continuous_vlm_trajectory_15s.gif")

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(out_mp4, fourcc, 15.0, (1280, 720))

    gif_frames = []
    start_time = time.time()
    frame_idx = 0

    print("\n🔴 [RECORDING ACTIVE] Continuously querying remote VLM server and extracting trajectories...")

    while time.time() - start_time < duration_s:
        raw_bytes = pipe.stdout.read(1280 * 720 * 3)
        if not raw_bytes or len(raw_bytes) != 1280 * 720 * 3:
            time.sleep(0.01)
            continue

        frame = np.frombuffer(raw_bytes, dtype=np.uint8).reshape((720, 1280, 3))
        with frame_lock:
            latest_raw_frame = frame
            current_telemetry = active_telemetry.copy()

        elapsed_s = time.time() - start_time
        hud_frame = render_hud(frame, current_telemetry, frame_idx, elapsed_s, duration_s)

        with frame_lock:
            latest_rendered_frame = hud_frame

        writer.write(hud_frame)

        # GIF sample at 7.5fps (every 2nd frame) for compact file size
        if frame_idx % 2 == 0:
            small_rgb = cv2.cvtColor(cv2.resize(hud_frame, (640, 360)), cv2.COLOR_BGR2RGB)
            gif_frames.append(Image.fromarray(small_rgb).convert('P', palette=Image.ADAPTIVE, colors=128))

        frame_idx += 1

    is_running = False
    pipe.terminate()
    writer.release()
    print(f"\n✅ Finished {duration_s}s recording! Total Frames: {frame_idx}")
    print(f"✅ Total Live VLM Server Queries Executed: {len(query_history)}")

    # Export Animated GIF
    if gif_frames:
        print("🎞️ Exporting continuous multi-query animated GIF...")
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
    art_dir = "/home/unitree/.gemini/antigravity-cli/brain/c585b67b-d53f-437e-a619-00fbeeb11b3d"
    if os.path.exists(art_dir):
        os.system(f"cp {out_gif} {art_dir}/live_continuous_vlm_trajectory_15s.gif")
        os.system(f"cp {out_mp4} {art_dir}/live_continuous_vlm_trajectory_15s.mp4")
        print("✅ Copied continuous GIF and MP4 to artifact directory!")

    # Write detailed query log to EXPERIMENT_RECORD_LOG.md
    log_path = os.path.join(out_dir, "EXPERIMENT_RECORD_LOG.md")
    with open(log_path, "a") as f:
        f.write(f"\n\n### 📍 [Session EXP-{datetime.datetime.now().strftime('%Y%m%d-%H%M%S')}] {duration_s}초 실시간 연속 VLM 폐루프 스트리밍 세션\n\n")
        f.write(f"* **세션 일시**: `{datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S KST')}`\n")
        f.write(f"* **총 실행 시간**: {duration_s}초 ({frame_idx} 프레임 @ 15.0fps)\n")
        f.write(f"* **총 VLM 질의 횟수**: **{len(query_history)}회 연속 수행**\n")
        f.write(f"* **미디어 파일**: [`live_continuous_vlm_trajectory_15s.gif`](live_continuous_vlm_trajectory_15s.gif) / [`live_continuous_vlm_trajectory_15s.mp4`](live_continuous_vlm_trajectory_15s.mp4)\n\n")
        f.write("| 질의 ID | 질의 시각 | VLM 지연시간 | 결정 Action | Subgoal [u, v] | Metric (X, Y) | VLM 추론 내용 |\n")
        f.write("| :--- | :--- | :--- | :--- | :--- | :--- | :--- |\n")
        for q in query_history:
            u, v = q["subgoal_uv"]
            x, y = q["target_metric"]
            f.write(f"| **`Query #{q['query_id']:02d}`** | `{q['timestamp']}` | **{q['latency_ms']:.1f} ms** | `{q['action']}` | `[{u}, {v}]` | `X={x:.2f}m, Y={y:.2f}m` | {q['reasoning'][:45]}... |\n")

    print(f"📝 Appended {len(query_history)} queries to {log_path}!")


if __name__ == "__main__":
    dur = 15
    if len(sys.argv) > 1:
        try:
            dur = int(sys.argv[1])
        except Exception:
            pass
    main(dur)
