from __future__ import annotations

from teleop.core.command_frame import CommandLoopDiagnostics
from teleop.core.robot_frame import DualArmRobotFeedback, DualArmRobotTarget, RobotArmFeedback, RobotArmTarget
from teleop.safety.state_machine import SafetyDecision, SafetyState
from teleop.ui.snapshot import ArmVisualizationSnapshot, LatestSnapshotStore, TeleopVisualizationSnapshot, compute_error_norm_mm
from teleop.ui.snapshot_builder import build_visualization_snapshot
from teleop.ui.ui_config import UIConfig


def _minimal_snapshot(timestamp_ns: int) -> TeleopVisualizationSnapshot:
    return TeleopVisualizationSnapshot(
        timestamp_ns=timestamp_ns,
        left=ArmVisualizationSnapshot(),
        right=ArmVisualizationSnapshot(),
    )


def test_compute_error_norm_mm_returns_expected_value() -> None:
    err = compute_error_norm_mm((0.0, 0.0, 0.0), (3.0, 4.0, 0.0))
    assert err == 5.0


def test_latest_snapshot_store_set_get_clear_and_latest_wins() -> None:
    store = LatestSnapshotStore()

    snap1 = _minimal_snapshot(1)
    snap2 = _minimal_snapshot(2)

    store.set(snap1)
    assert store.get() == snap1

    store.set(snap2)
    assert store.get() == snap2

    store.clear()
    assert store.get() is None


def test_snapshot_builder_handles_missing_inputs() -> None:
    snapshot = build_visualization_snapshot(
        timestamp_ns=123,
        target=None,
        feedback=None,
        safety_decision=None,
        command_diagnostics=None,
        logging_stats=None,
    )

    assert snapshot.timestamp_ns == 123
    assert snapshot.left.target_xyz_mm is None
    assert snapshot.left.feedback_xyz_mm is None
    assert snapshot.right.target_xyz_mm is None
    assert snapshot.right.feedback_xyz_mm is None
    assert snapshot.left.error_norm_mm is None
    assert snapshot.right.error_norm_mm is None
    assert snapshot.safety_state == "UNKNOWN"


def test_snapshot_builder_computes_error_norm_when_target_and_feedback_exist() -> None:
    target = DualArmRobotTarget(
        left=RobotArmTarget(
            position_xyz=(0.0, 0.0, 0.0),
            orientation_abc=(0.0, 0.0, 0.0),
            valid=True,
            reason="",
        ),
        right=None,
    )
    feedback = DualArmRobotFeedback(
        left=RobotArmFeedback(
            position_xyz=(3.0, 4.0, 0.0),
            orientation_abc=(0.0, 0.0, 0.0),
            valid=True,
        ),
        right=None,
    )
    decision = SafetyDecision(
        state=SafetyState.TELEOP_ACTIVE,
        allow_motion=True,
        left_allowed=True,
        right_allowed=False,
        left_reason="ok",
        right_reason="disabled",
        global_reason="ok",
        safe_target=target,
    )
    diagnostics = CommandLoopDiagnostics(
        loop_rate_hz=100.0,
        dt_ms=10.0,
        target_age_ms=20.0,
        used_zero_order_hold=False,
        limited=False,
        limit_reason="ok",
        sequence_id=1,
    )

    snapshot = build_visualization_snapshot(
        timestamp_ns=456,
        target=target,
        feedback=feedback,
        safety_decision=decision,
        command_diagnostics=diagnostics,
        pico_connected=True,
        robot_connected=True,
    )

    assert snapshot.left.error_norm_mm == 5.0
    assert snapshot.left.active is True
    assert snapshot.right.active is False
    assert snapshot.command_loop_dt_ms == 10.0
    assert snapshot.target_age_ms == 20.0


def test_ui_config_defaults() -> None:
    cfg = UIConfig()
    assert cfg.enabled is False
    assert cfg.update_hz == 20.0
    assert cfg.camera_distance_mm == 5000.0
    assert cfg.grid_size_mm == 4000.0
    assert cfg.grid_spacing_mm == 200.0
    assert cfg.auto_center is True
