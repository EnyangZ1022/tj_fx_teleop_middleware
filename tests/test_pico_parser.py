import json

from teleop.core.pico_frame import PicoRawFrame
from teleop.input.pico_receiver import PicoReceiver, parse_pose_str, parse_state_json


def test_parse_pose_str() -> None:
    pose = parse_pose_str("1,2,3,0,0,0,1")
    assert pose.as_tuple() == (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)


def test_parse_state_json_double_encoded() -> None:
    inner = {
        "timeStampNs": 123456789,
        "Head": {
            "pose": "0.0,1.0,2.0,0.0,0.0,0.0,1.0",
        },
        "Controller": {
            "left": {
                "pose": "0.1,0.2,0.3,0.0,0.0,0.0,1.0",
                "trigger": 0.6,
                "grip": 0.7,
                "axisX": -0.2,
                "axisY": 0.5,
                "axisClick": True,
                "primaryButton": True,
                "secondaryButton": False,
                "menuButton": False,
            },
            "right": {
                "pose": "-0.1,-0.2,-0.3,0.0,0.0,0.0,1.0",
                "trigger": 0.4,
                "grip": 0.3,
                "axisX": 0.9,
                "axisY": -0.6,
                "axisClick": False,
                "primaryButton": False,
                "secondaryButton": True,
                "menuButton": True,
            },
        },
    }

    raw_json = json.dumps({"value": json.dumps(inner)})
    frame = parse_state_json(
        raw_json=raw_json,
        dev_id="pico_device_01",
        frame_id=5,
        pc_receive_time_ns=999,
    )

    assert isinstance(frame, PicoRawFrame)
    assert frame is not None
    assert frame.device_id == "pico_device_01"
    assert frame.frame_id == 5
    assert frame.pico_timestamp_ns == 123456789
    assert frame.pc_receive_time_ns == 999

    assert frame.left_ctrl.axis_click is True
    assert frame.right_ctrl.axis_click is False
    assert frame.left_ctrl.trigger == 0.6
    assert frame.right_ctrl.grip == 0.3


def test_pico_receiver_emits_receiver_timing_payload() -> None:
    inner = {
        "timeStampNs": 1234,
        "Head": {"pose": "0,0,0,0,0,0,1"},
        "Controller": {
            "left": {"pose": "0,0,0,0,0,0,1"},
            "right": {"pose": "0,0,0,0,0,0,1"},
        },
    }
    raw_json = json.dumps({"value": json.dumps(inner)})

    payloads: list[dict[str, object]] = []

    receiver = PicoReceiver(on_receiver_timing=lambda payload: payloads.append(dict(payload)))
    receiver._on_state_json(raw_json, "dev_a")

    assert len(payloads) == 1
    payload = payloads[0]
    assert payload["receiver_seq"] == 1
    assert isinstance(payload.get("pc_receive_perf_ns"), int)
    assert isinstance(payload.get("pc_receive_wall_ns"), int)
    assert payload["pico_source_timestamp_ns"] == 1234
    assert payload["frame_seq"] == 1
    assert isinstance(payload.get("parse_duration_ms"), float)
    assert float(payload["parse_duration_ms"]) >= 0.0
    assert isinstance(payload.get("json_size_bytes"), int)
