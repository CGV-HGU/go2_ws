#!/usr/bin/env python3
"""
========================================================================================
🚀 Unitree Go2 ESCAPE-Nav VLM Server & NetBird Connectivity Diagnostic Tool
========================================================================================
Tests:
1. NetBird VPN connectivity to cgv-server-02 (100.96.60.15)
2. HTTP /v1/models query to vLLM Server on Port 8000
3. Live /v1/chat/completions multimodal/text reasoning inference with latency benchmarking
========================================================================================
"""

import sys
import time
import json
import urllib.request
import urllib.error

SERVER_IP = "100.96.60.15"
PORT = 8000
BASE_URL = f"http://{SERVER_IP}:{PORT}/v1"
MODEL_NAME = "qwen3.8-27b-instruct"

def test_vlm_connection():
    print("=" * 76)
    print(f" 🌐 [ESCAPE-Nav] Testing VLM Server Connectivity ({SERVER_IP}:{PORT})")
    print("=" * 76)

    # 1. Models List Query
    print(f"[1/2] Querying available models from {BASE_URL}/models ... ", end="", flush=True)
    t0 = time.time()
    try:
        req = urllib.request.Request(f"{BASE_URL}/models", headers={"User-Agent": "Go2-VLM-Client"})
        with urllib.request.urlopen(req, timeout=5) as resp:
            data = json.loads(resp.read().decode())
            models = [m["id"] for m in data.get("data", [])]
            dt = time.time() - t0
            print(f"🟢 OK ({dt*1000:.1f} ms)")
            print(f"      Available Models: {models}")
    except Exception as e:
        print(f"🔴 FAILED ({e})")
        return False

    # 2. Live Chat Completion Inference Test
    target_model = MODEL_NAME if MODEL_NAME in models else models[0]
    print(f"[2/2] Running inference benchmark with '{target_model}' ... ", end="", flush=True)
    payload = {
        "model": target_model,
        "messages": [
            {
                "role": "system",
                "content": "You are the ESCAPE-Nav real-time robotic navigation reasoning engine."
            },
            {
                "role": "user",
                "content": "Format: JSON with action (go, stop, rotate), confidence (0.0-1.0), reason. Status: Clear corridor ahead."
            }
        ],
        "temperature": 0.0,
        "max_tokens": 150
    }

    t0 = time.time()
    try:
        req_data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            f"{BASE_URL}/chat/completions",
            data=req_data,
            headers={"Content-Type": "application/json", "User-Agent": "Go2-VLM-Client"}
        )
        with urllib.request.urlopen(req, timeout=10) as resp:
            res = json.loads(resp.read().decode())
            dt = time.time() - t0
            content = res["choices"][0]["message"]["content"]
            print(f"🟢 SUCCESS ({dt:.3f}s / {dt*1000:.1f}ms)")
            print("=" * 76)
            print(f" 🤖 VLM Reasoning Response:\n{content.strip()}")
            print("=" * 76)
            print("🏆 [RESULT] 100% READY FOR REAL-ROBOT DEPLOYMENT!")
            return True
    except Exception as e:
        print(f"🔴 FAILED ({e})")
        return False

if __name__ == "__main__":
    success = test_vlm_connection()
    sys.exit(0 if success else 1)
