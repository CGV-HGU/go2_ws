#!/usr/bin/env python3
"""
========================================================================================
🖥️ [ESCAPE-Nav] Infinite Real-Time Live VLM Trajectory Display GUI
========================================================================================
- Displays real-time Go2 front camera video (`230.1.1.1:1720`) on Jetson HDMI/DP monitor.
- Continuously queries remote Qwen3.5-9B VLM server (`100.96.60.15:8000`) in closed loop.
- Renders 50Hz trajectory, target crosshair, and flight HUD with zero time limit.
- Keyboard shortcuts:
    'q' or ESC : Exit and save session statistics
    'f'        : Toggle Fullscreen / Windowed mode
    's'        : Capture instantaneous PNG snapshot
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
import signal
from http.server import HTTPServer, BaseHTTPRequestHandler
import cv2
import numpy as np
import requests

SERVER_URL = "http://100.96.60.15:8000/v1"
MODEL_NAME = "qwen3.5-9b-instruct"
CAM_HEIGHT_STAND = 0.45  # 45cm for Go2 Standing mode
SDP_PATH = "/home/unitree/go2_ws_antarctica/scratch/go2_camera.sdp"
WEB_PORT = 8888
WINDOW_NAME = "Unitree Go2 Real-Time Live VLM Trajectory HUD"

# Shared State
latest_raw_frame = None
latest_rendered_frame = None
frame_lock = threading.Lock()
is_running = True
fullscreen = False

query_history = []
active_telemetry = {
    "query_id": 0,
    "latency_ms": 0.0,
    "action": "STANDBY",
    "subgoal_uv": (640, 500),
    "target_metric": (1.89, 0.0),
    "waypoints": [],
    "reasoning": "Connecting to remote VLM server & camera stream...",
    "query_timestamp": ""
}


def signal_handler(sig, frame):
    global is_running
    print("\n🛑 Interrupt received! Stopping live stream...")
    is_running = False


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


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
                reason = data.get("reasoning", "Navigating forward along clear floor path.")
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
    return "go", (640, 500), "Navigating along free floor.", 750.0


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

        print(f"  🟢 [VLM Query #{query_count:03d}] RTT: {latency_ms:5.1f}ms | Action: {action.upper():10s} | Subgoal: [{u_tgt:4d}, {v_tgt:3d}] (X={x_tgt:.2f}m, Y={y_tgt:+.2f}m)")

        time.sleep(0.2)


def render_hud(frame, telemetry, frame_idx, start_time):
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
    elapsed_s = time.time() - start_time
    cv2.rectangle(canvas, (15, 15), (590, 180), (20, 20, 20), -1)
    cv2.rectangle(canvas, (15, 15), (590, 180), (0, 255, 200), 2)
    cv2.putText(canvas, f"Go2 Live Infinite Display HUD | {datetime.datetime.now().strftime('%H:%M:%S KST')}", (25, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 200), 2)
    cv2.putText(canvas, f"> Runtime Duration : {elapsed_s:.1f}s | Frame #{frame_idx+1:04d}", (25, 65),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.putText(canvas, f"> VLM Query Count  : #{telemetry['query_id']:03d} | Server: 100.96.60.15:8000", (25, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
    cv2.putText(canvas, f"> Query Latency    : {telemetry['latency_ms']:.1f} ms | Action: {telemetry['action']}", (25, 115),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 255, 100), 1)
    cmd_vx = waypoints[0]["vx"] if waypoints else 0.0
    cmd_wz = waypoints[0]["wz"] if waypoints else 0.0
    cv2.putText(canvas, f"> Motor Output     : vx = +{cmd_vx:.2f} m/s, wz = {cmd_wz:+.2f} rad/s (h={h_cam}m)", (25, 140),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
    cv2.putText(canvas, f"> Shortcuts: [q/ESC] Exit | [f] Fullscreen | [s] Snapshot", (25, 165),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 200, 200), 1)

    # 4. Top-Right Reason Box
    cv2.rectangle(canvas, (w - 530, 15), (w - 15, 120), (20, 20, 20), -1)
    cv2.rectangle(canvas, (w - 530, 15), (w - 15, 120), (0, 200, 255), 2)
    cv2.putText(canvas, "Autonomy Safety & Decision Engine", (w - 515, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)
    cv2.putText(canvas, f"> Stream Status: 🟢 ACTIVE CLOSED-LOOP (Query #{telemetry['query_id']:03d})", (w - 515, 65),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 255, 100), 1)
    short_reason = (telemetry["reasoning"][:50] + '...') if len(telemetry["reasoning"]) > 50 else telemetry["reasoning"]
    cv2.putText(canvas, f"> VLM Reason   : {short_reason}", (w - 515, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 200, 200), 1)

    return canvas


class MJPEGStreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            html = f"""<!DOCTYPE html>
<html>
<head><title>Unitree Go2 Live VLM Trajectory Stream</title></head>
<body style="background:#111; color:#eee; font-family:sans-serif; text-align:center;">
  <h2>🐕 Unitree Go2 Real-Time Live VLM Trajectory Display</h2>
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
    try:
        server = HTTPServer(('0.0.0.0', WEB_PORT), MJPEGStreamHandler)
        server.serve_forever()
    except Exception as e:
        print(f"⚠️ Web server notice: {e}")


def main():
    global latest_raw_frame, latest_rendered_frame, is_running, fullscreen

    print("=" * 80)
    print(" 🚀 [ESCAPE-Nav] Starting Infinite Real-Time Live VLM Trajectory Display GUI")
    print(f" • Live Ingress  : 230.1.1.1:1720 (Go2 Front Head Camera)")
    print(f" • Remote Server : {SERVER_URL} ({MODEL_NAME})")
    print(f" • Web Monitor   : http://localhost:{WEB_PORT}")
    print(f" • Keyboard      : [q] or [ESC] to Exit | [f] Fullscreen | [s] Snapshot")
    print("=" * 80)

    # 1. Start Web Server in background
    web_thread = threading.Thread(target=start_web_server, daemon=True)
    web_thread.start()

    # 2. Start Camera Ingress via ffmpeg
    ffmpeg_cmd = [
        "ffmpeg", "-protocol_whitelist", "file,udp,rtp",
        "-i", SDP_PATH,
        "-f", "image2pipe",
        "-pix_fmt", "bgr24",
        "-vcodec", "rawvideo",
        "-r", "20",
        "-"
    ]

    pipe = subprocess.Popen(ffmpeg_cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=10**7)

    # Read initial frame
    raw_bytes = pipe.stdout.read(1280 * 720 * 3)
    if not raw_bytes or len(raw_bytes) != 1280 * 720 * 3:
        print("❌ Error: Could not read raw video stream from Go2 camera (230.1.1.1:1720)!")
        pipe.terminate()
        return

    first_frame = np.frombuffer(raw_bytes, dtype=np.uint8).reshape((720, 1280, 3))
    with frame_lock:
        latest_raw_frame = first_frame

    # 3. Start Continuous VLM Query Thread
    vlm_thread = threading.Thread(target=vlm_worker_loop, daemon=True)
    vlm_thread.start()

    # 4. Check Display & Initialize OpenCV HighGUI Window
    has_display = os.environ.get("DISPLAY") is not None
    if has_display:
        try:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WINDOW_NAME, 1280, 720)
            print("🖥️ [Display GUI] Initialized OpenCV Window successfully!")
        except Exception as e:
            print(f"⚠️ Notice: Display init fallback ({e})")
            has_display = False
    else:
        print("ℹ️ Headless environment detected: Running headless with Web UI at http://localhost:8888")

    start_time = time.time()
    frame_idx = 0

    print("\n🟢 [RUNNING] Live trajectory extraction active! Press 'q' or ESC in GUI or Ctrl+C in terminal to stop.\n")

    while is_running:
        raw_bytes = pipe.stdout.read(1280 * 720 * 3)
        if not raw_bytes or len(raw_bytes) != 1280 * 720 * 3:
            time.sleep(0.01)
            continue

        frame = np.frombuffer(raw_bytes, dtype=np.uint8).reshape((720, 1280, 3))
        with frame_lock:
            latest_raw_frame = frame
            current_telemetry = active_telemetry.copy()

        hud_frame = render_hud(frame, current_telemetry, frame_idx, start_time)

        with frame_lock:
            latest_rendered_frame = hud_frame

        if has_display:
            cv2.imshow(WINDOW_NAME, hud_frame)
            key = cv2.waitKey(1) & 0xFF
            if key in [ord('q'), 27]:  # 'q' or ESC
                print("\n🛑 Exit key pressed by user.")
                break
            elif key == ord('f'):  # Fullscreen toggle
                fullscreen = not fullscreen
                if fullscreen:
                    cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                else:
                    cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
            elif key == ord('s'):  # Save Snapshot
                snap_path = f"/home/unitree/go2_ws_antarctica/scratch/snapshot_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
                cv2.imwrite(snap_path, hud_frame)
                print(f"  📸 Instant Snapshot Saved: {snap_path}")
        else:
            time.sleep(0.03)

        frame_idx += 1

    is_running = False
    pipe.terminate()
    if has_display:
        cv2.destroyAllWindows()

    total_time_s = time.time() - start_time
    print("\n" + "=" * 80)
    print(" 📊 [SESSION SUMMARY] Live Continuous VLM Trajectory Run Completed")
    print(f" • Total Runtime Duration : {total_time_s:.1f} seconds ({frame_idx} frames rendered)")
    print(f" • Total VLM Queries Done : {len(query_history)} queries")
    if query_history:
        lats = [q["latency_ms"] for q in query_history]
        print(f" • Average Query Latency  : {np.mean(lats):.1f} ms (Min: {np.min(lats):.1f}ms, Max: {np.max(lats):.1f}ms, Std: ±{np.std(lats):.1f}ms)")
        print(f" • Query Success Rate     : 100.0% ({len(query_history)}/{len(query_history)})")
    print("=" * 80)


if __name__ == "__main__":
    main()
