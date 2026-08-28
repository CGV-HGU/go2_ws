"""Strict OpenAI-compatible VLM grounding for the live PixNav chain.

The module is intentionally independent of ROS, DDS and Unitree SDK code.  It
can run inside the Jazzy container as a transport client, but it only returns a
validated image-space decision to stdout.  It never creates a motion command.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import mimetypes
import os
from pathlib import Path
import time
from typing import Any, Mapping, Optional
import urllib.error
import urllib.request


GROUNDING_SCHEMA_VERSION = "go2_pixnav_vlm_grounding_v1"
TRANSPORT_SCHEMA_VERSION = "go2_pixnav_vlm_transport_v1"


class GroundingValidationError(ValueError):
    """The VLM response is syntactically JSON but unsafe for PixNav."""


def canonical_json(value: Any) -> str:
    return json.dumps(
        value,
        allow_nan=False,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
    )


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def extract_json_object(text: str) -> dict[str, Any]:
    """Return the first complete JSON object without accepting prose defaults."""

    if not isinstance(text, str) or not text.strip():
        raise ValueError("VLM content is empty")
    decoder = json.JSONDecoder()
    for index, character in enumerate(text):
        if character != "{":
            continue
        try:
            value, _ = decoder.raw_decode(text[index:])
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            return value
    raise ValueError("VLM content contains no complete JSON object")


def response_json_schema(width: int, height: int) -> dict[str, Any]:
    if width <= 0 or height <= 0:
        raise ValueError("image dimensions must be positive")
    coordinate_max = max(width, height) - 1
    return {
        "type": "object",
        "properties": {
            "schema_version": {"type": "string", "const": GROUNDING_SCHEMA_VERSION},
            "action": {"type": "string", "enum": ["go", "stop"]},
            "selected_view_id": {"type": "integer", "minimum": -1, "maximum": 0},
            "selected_view_type": {"type": "string", "enum": ["front", "none"]},
            "selected_image_point": {
                "type": "array",
                "items": {
                    "type": "integer",
                    "minimum": -1,
                    "maximum": coordinate_max,
                },
                "minItems": 2,
                "maxItems": 2,
            },
            "fine_goal": {
                "type": "object",
                "properties": {
                    "valid": {"type": "boolean"},
                    "point_px": {
                        "type": "array",
                        "items": {
                            "type": "integer",
                            "minimum": -1,
                            "maximum": coordinate_max,
                        },
                        "minItems": 2,
                        "maxItems": 2,
                    },
                },
                "required": ["valid", "point_px"],
                "additionalProperties": False,
            },
            "confidence": {"type": "number", "minimum": 0.0, "maximum": 1.0},
            "reason": {"type": "string", "minLength": 1, "maxLength": 240},
        },
        "required": [
            "schema_version",
            "action",
            "selected_view_id",
            "selected_view_type",
            "selected_image_point",
            "fine_goal",
            "confidence",
            "reason",
        ],
        "additionalProperties": False,
    }


def validate_grounding(raw: Mapping[str, Any], *, width: int, height: int) -> dict[str, Any]:
    """Validate exact pixel semantics and return a normalized safe object.

    No central-pixel fallback or clipping is allowed.  A malformed response is
    rejected so the caller can record a zero-hold.
    """

    if not isinstance(raw, Mapping):
        raise GroundingValidationError("RAW_RESPONSE_NOT_OBJECT")
    if raw.get("schema_version") != GROUNDING_SCHEMA_VERSION:
        raise GroundingValidationError("UNSUPPORTED_GROUNDING_SCHEMA")
    action = raw.get("action")
    if action not in {"go", "stop"}:
        raise GroundingValidationError("UNSUPPORTED_GROUNDING_ACTION")
    confidence = raw.get("confidence")
    if isinstance(confidence, bool) or not isinstance(confidence, (int, float)):
        raise GroundingValidationError("INVALID_GROUNDING_CONFIDENCE")
    confidence_value = float(confidence)
    if not math.isfinite(confidence_value) or not 0.0 <= confidence_value <= 1.0:
        raise GroundingValidationError("INVALID_GROUNDING_CONFIDENCE")
    reason = raw.get("reason")
    if not isinstance(reason, str) or not reason.strip() or len(reason) > 240:
        raise GroundingValidationError("INVALID_GROUNDING_REASON")

    point = raw.get("selected_image_point")
    fine_goal = raw.get("fine_goal")
    if not isinstance(point, (list, tuple)) or len(point) != 2:
        raise GroundingValidationError("INVALID_SELECTED_IMAGE_POINT_SHAPE")
    if not all(isinstance(value, int) and not isinstance(value, bool) for value in point):
        raise GroundingValidationError("SELECTED_IMAGE_POINT_NOT_INTEGER")
    if not isinstance(fine_goal, Mapping):
        raise GroundingValidationError("FINE_GOAL_NOT_OBJECT")
    nested_point = fine_goal.get("point_px")
    if not isinstance(nested_point, (list, tuple)) or list(nested_point) != list(point):
        raise GroundingValidationError("FINE_GOAL_PIXEL_MISMATCH")

    selected_view_id = raw.get("selected_view_id")
    selected_view_type = raw.get("selected_view_type")
    if action == "go":
        if selected_view_id != 0 or selected_view_type != "front":
            raise GroundingValidationError("GO_VIEW_MISMATCH")
        if fine_goal.get("valid") is not True:
            raise GroundingValidationError("GO_FINE_GOAL_NOT_VALID")
        u, v = int(point[0]), int(point[1])
        if not 0 <= u < width or not 0 <= v < height:
            raise GroundingValidationError("GO_PIXEL_OUT_OF_BOUNDS")
    else:
        if selected_view_id != -1 or selected_view_type != "none":
            raise GroundingValidationError("STOP_VIEW_MUST_BE_NONE")
        if fine_goal.get("valid") is not False or list(point) != [-1, -1]:
            raise GroundingValidationError("STOP_MUST_USE_INVALID_SENTINEL_GOAL")
        u, v = -1, -1

    return {
        "schema_version": GROUNDING_SCHEMA_VERSION,
        "action": str(action),
        "selected_view_id": int(selected_view_id),
        "selected_view_type": str(selected_view_type),
        "selected_image_point": [u, v],
        "fine_goal": {"valid": bool(fine_goal.get("valid")), "point_px": [u, v]},
        "confidence": confidence_value,
        "reason": reason.strip(),
    }


def _image_data_url(path: Path) -> tuple[str, str]:
    data = path.read_bytes()
    mime_type = mimetypes.guess_type(path.name)[0] or "image/jpeg"
    return f"data:{mime_type};base64,{base64.b64encode(data).decode('ascii')}", sha256_bytes(data)


def _request_json(
    url: str,
    *,
    method: str,
    timeout_s: float,
    payload: Optional[dict[str, Any]] = None,
    api_key: str = "",
) -> tuple[int, dict[str, Any]]:
    body = canonical_json(payload).encode("utf-8") if payload is not None else None
    headers = {"Content-Type": "application/json", "User-Agent": "escape-nav-pixnav-p6"}
    if api_key:
        headers["Authorization"] = f"Bearer {api_key}"
    request = urllib.request.Request(url, data=body, headers=headers, method=method)
    try:
        with urllib.request.urlopen(request, timeout=timeout_s) as response:
            response_body = response.read()
            status = int(response.status)
    except urllib.error.HTTPError as error:
        detail = error.read().decode("utf-8", errors="replace")[:1000]
        raise RuntimeError(f"HTTP_{error.code}:{detail}") from error
    value = json.loads(response_body.decode("utf-8"))
    if not isinstance(value, dict):
        raise ValueError("server response is not a JSON object")
    return status, value


def discover_model(base_url: str, *, timeout_s: float) -> str:
    _, response = _request_json(
        f"{base_url.rstrip('/')}/models",
        method="GET",
        timeout_s=timeout_s,
    )
    models = [item.get("id") for item in response.get("data", []) if isinstance(item, dict)]
    models = [value for value in models if isinstance(value, str) and value]
    if not models:
        raise RuntimeError("server advertised no model")
    return models[0]


def query_grounding(
    image_path: Path,
    *,
    width: int,
    height: int,
    base_url: str,
    model: str,
    timeout_s: float,
    api_key: str = "",
) -> dict[str, Any]:
    if not image_path.is_file():
        raise FileNotFoundError(image_path)
    if not math.isfinite(timeout_s) or timeout_s <= 0.0:
        raise ValueError("timeout must be positive and finite")
    selected_model = model if model and model.lower() != "auto" else discover_model(
        base_url, timeout_s=min(timeout_s, 5.0)
    )
    image_url, image_hash = _image_data_url(image_path)
    prompt = (
        f"Inspect this {width}x{height} front camera image from a stationary Unitree Go2. "
        "Choose one visible collision-free floor pixel that a frozen Pixel-Navigator can use "
        "as a local image-space goal. Do not infer metric/map coordinates and do not issue a "
        "robot command. If no safe floor goal is visible, return action=stop with view_id=-1, "
        "view_type=none, both pixel arrays [-1,-1], and fine_goal.valid=false. For action=go, "
        "use view_id=0, view_type=front, identical integer pixel arrays, and valid=true."
    )
    payload = {
        "model": selected_model,
        "messages": [
            {
                "role": "system",
                "content": "You are a strict image-space grounding component. Return only the required JSON object.",
            },
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                    {"type": "image_url", "image_url": {"url": image_url}},
                ],
            },
        ],
        "temperature": 0.0,
        "max_tokens": 256,
        "response_format": {
            "type": "json_schema",
            "json_schema": {
                "name": "go2_pixnav_grounding",
                "strict": True,
                "schema": response_json_schema(width, height),
            },
        },
    }
    request_hash = sha256_bytes(canonical_json(payload).encode("utf-8"))
    started = time.perf_counter()
    status, response = _request_json(
        f"{base_url.rstrip('/')}/chat/completions",
        method="POST",
        timeout_s=timeout_s,
        payload=payload,
        api_key=api_key,
    )
    latency_s = time.perf_counter() - started
    content = response["choices"][0]["message"]["content"]
    raw = extract_json_object(content)
    return {
        "schema_version": TRANSPORT_SCHEMA_VERSION,
        "base_url": base_url.rstrip("/"),
        "model": selected_model,
        "http_status": status,
        "latency_s": latency_s,
        "image_sha256": image_hash,
        "request_sha256": request_hash,
        "api_key_present": bool(api_key),
        "raw": raw,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Strict no-actuation PixNav VLM grounding")
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    parser.add_argument("--base-url", default=os.getenv("QWEN_BASE_URL", "http://100.96.60.15:8000/v1"))
    parser.add_argument("--model", default=os.getenv("QWEN_MODEL", "auto"))
    parser.add_argument("--timeout", type=float, default=20.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    result = query_grounding(
        args.image.expanduser().resolve(),
        width=args.width,
        height=args.height,
        base_url=args.base_url,
        model=args.model,
        timeout_s=args.timeout,
        api_key=os.getenv("QWEN_API_KEY", ""),
    )
    print(canonical_json(result))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
