from teleop.core.pico_frame import PicoControllerState, PicoRawFrame
from teleop.core.pose import Pose7
from teleop.core.teleop_frame import TeleopFrame
from teleop.input.pico_mapping import PicoInputMapper


VALID_POSE = Pose7.from_tuple((0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0))
ZERO_POSE = Pose7.from_tuple((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))


def make_ctrl(
    *,
    pose: Pose7 = VALID_POSE,
    trigger: float = 0.5,
    grip: float = 0.5,
    axis_x: float = 0.0,
    axis_y: float = 0.0,
    axis_click: bool = False,
    primary_button: bool = False,
    secondary_button: bool = False,
    menu_button: bool = False,
) -> PicoControllerState:
    return PicoControllerState(
        pose=pose,
        trigger=trigger,
        grip=grip,
        axis_x=axis_x,
        axis_y=axis_y,
        axis_click=axis_click,
        primary_button=primary_button,
        secondary_button=secondary_button,
        menu_button=menu_button,
    )


def make_frame(
    *,
    frame_id: int = 1,
    left_ctrl: PicoControllerState | None = None,
    right_ctrl: PicoControllerState | None = None,
) -> PicoRawFrame:
    return PicoRawFrame(
        frame_id=frame_id,
        device_id="pico_test",
        pico_timestamp_ns=111,
        pc_receive_time_ns=222,
        head_pose=VALID_POSE,
        left_ctrl=left_ctrl if left_ctrl is not None else make_ctrl(),
        right_ctrl=right_ctrl if right_ctrl is not None else make_ctrl(),
    )


def test_map_frame_returns_teleop_frame_and_enable_mapping() -> None:
    mapper = PicoInputMapper()
    frame = make_frame(
        left_ctrl=make_ctrl(grip=0.81),
        right_ctrl=make_ctrl(grip=0.8),
    )

    mapped = mapper.map_frame(frame)

    assert isinstance(mapped, TeleopFrame)
    assert mapped.left.enable is True
    assert mapped.right.enable is False


def test_trigger_to_gripper_position_and_clamp() -> None:
    mapper = PicoInputMapper()
    frame = make_frame(
        left_ctrl=make_ctrl(trigger=0.0),
        right_ctrl=make_ctrl(trigger=1.0),
    )
    mapped = mapper.map_frame(frame)

    assert mapped.left.gripper_position == 1.0
    assert mapped.right.gripper_position == 0.0

    frame_clamped = make_frame(
        frame_id=2,
        left_ctrl=make_ctrl(trigger=-1.0),
        right_ctrl=make_ctrl(trigger=2.0),
    )
    mapped_clamped = mapper.map_frame(frame_clamped)

    assert mapped_clamped.left.gripper_position == 1.0
    assert mapped_clamped.right.gripper_position == 0.0


def test_gripper_deadband_behavior() -> None:
    mapper = PicoInputMapper(gripper_deadband=0.01)

    frame1 = make_frame(frame_id=1, left_ctrl=make_ctrl(trigger=0.5))
    frame2 = make_frame(frame_id=2, left_ctrl=make_ctrl(trigger=0.505))
    frame3 = make_frame(frame_id=3, left_ctrl=make_ctrl(trigger=0.55))

    mapped1 = mapper.map_frame(frame1)
    mapped2 = mapper.map_frame(frame2)
    mapped3 = mapper.map_frame(frame3)

    assert mapped1.left.gripper_changed is True
    assert mapped2.left.gripper_changed is False
    assert mapped3.left.gripper_changed is True


def test_invalid_zero_pose_maps_to_invalid_arm_input() -> None:
    mapper = PicoInputMapper()
    frame = make_frame(
        left_ctrl=make_ctrl(pose=ZERO_POSE),
        right_ctrl=make_ctrl(pose=VALID_POSE),
    )

    mapped = mapper.map_frame(frame)

    assert mapped.left.valid is False
    assert mapped.left.pose_pico is None
    assert mapped.right.valid is True
    assert mapped.right.pose_pico is not None


def test_axis_click_and_reserved_button_requests() -> None:
    mapper = PicoInputMapper()
    frame = make_frame(
        left_ctrl=make_ctrl(axis_click=True, secondary_button=True),
        right_ctrl=make_ctrl(axis_click=False, primary_button=True, menu_button=True),
    )

    mapped = mapper.map_frame(frame)

    assert mapped.left.axis_click is True
    assert mapped.right.axis_click is False
    assert mapped.start_pause_requested is True
    assert mapped.cancel_requested is True
    assert mapped.calibration_requested is True
