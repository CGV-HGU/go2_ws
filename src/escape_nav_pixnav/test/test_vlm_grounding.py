import pytest

from escape_nav_pixnav.vlm_grounding import (
    GROUNDING_SCHEMA_VERSION,
    GroundingValidationError,
    extract_json_object,
    response_json_schema,
    validate_grounding,
)


def valid_go():
    return {
        "schema_version": GROUNDING_SCHEMA_VERSION,
        "action": "go",
        "selected_view_id": 0,
        "selected_view_type": "front",
        "selected_image_point": [640, 600],
        "fine_goal": {"valid": True, "point_px": [640, 600]},
        "confidence": 0.8,
        "reason": "visible free floor",
    }


def test_valid_go_grounding_preserves_exact_pixel():
    result = validate_grounding(valid_go(), width=1280, height=720)
    assert result["selected_image_point"] == [640, 600]
    assert result["fine_goal"]["point_px"] == [640, 600]


def test_mismatched_nested_pixel_is_rejected_without_sanitizing():
    value = valid_go()
    value["fine_goal"]["point_px"] = [641, 600]
    with pytest.raises(GroundingValidationError, match="FINE_GOAL_PIXEL_MISMATCH"):
        validate_grounding(value, width=1280, height=720)


def test_out_of_bounds_pixel_is_rejected_without_clipping():
    value = valid_go()
    value["selected_image_point"] = [1280, 600]
    value["fine_goal"]["point_px"] = [1280, 600]
    with pytest.raises(GroundingValidationError, match="GO_PIXEL_OUT_OF_BOUNDS"):
        validate_grounding(value, width=1280, height=720)


def test_valid_stop_requires_explicit_invalid_sentinel():
    value = {
        "schema_version": GROUNDING_SCHEMA_VERSION,
        "action": "stop",
        "selected_view_id": -1,
        "selected_view_type": "none",
        "selected_image_point": [-1, -1],
        "fine_goal": {"valid": False, "point_px": [-1, -1]},
        "confidence": 0.7,
        "reason": "no safe floor goal",
    }
    assert validate_grounding(value, width=1280, height=720)["action"] == "stop"


def test_json_extraction_accepts_fenced_object_but_no_plain_text_default():
    assert extract_json_object("```json\n{\"action\":\"stop\"}\n```")["action"] == "stop"
    with pytest.raises(ValueError, match="no complete JSON object"):
        extract_json_object("move forward")


def test_response_schema_is_closed_and_dimension_bounded():
    schema = response_json_schema(1280, 720)
    assert schema["additionalProperties"] is False
    point_items = schema["properties"]["selected_image_point"]["items"]
    assert point_items["minimum"] == -1
    assert point_items["maximum"] == 1279
