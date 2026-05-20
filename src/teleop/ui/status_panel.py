from __future__ import annotations

from PySide6.QtWidgets import QFormLayout, QLabel, QSizePolicy, QWidget

from teleop.ui.snapshot import TeleopVisualizationSnapshot


class TeleopStatusPanel(QWidget):
    """Compact status panel for Stage 8 diagnostic UI."""

    def __init__(self, parent=None):
        super().__init__(parent=parent)

        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)

        self._labels: dict[str, QLabel] = {}
        layout = QFormLayout(self)

        for key, title in _STATUS_ROWS:
            value_label = QLabel("-")
            value_label.setTextInteractionFlags(value_label.textInteractionFlags())
            self._labels[key] = value_label
            layout.addRow(title, value_label)

    def update_from_snapshot(self, snapshot: TeleopVisualizationSnapshot) -> None:
        self._set_text("pico_connected", _format_bool(snapshot.pico_connected))
        self._set_text("robot_connected", _format_bool(snapshot.robot_connected))
        self._set_text("safety_state", str(snapshot.safety_state))
        self._set_text("global_status", str(snapshot.global_status) if snapshot.global_status else "-")
        self._set_text("teleop_mode", str(snapshot.teleop_mode) if snapshot.teleop_mode else "-")
        self._set_text("orientation_tracking", _format_bool(snapshot.orientation_tracking_enabled))
        self._set_text(
            "orientation_relative_mode",
            str(snapshot.orientation_relative_mode) if snapshot.orientation_relative_mode else "-",
        )

        self._set_text("left_calibrated", _format_bool(snapshot.left.calibrated))
        self._set_text("left_active", _format_bool(snapshot.left.active))
        self._set_text("left_enable", _format_bool(snapshot.enable_left))

        self._set_text("right_calibrated", _format_bool(snapshot.right.calibrated))
        self._set_text("right_active", _format_bool(snapshot.right.active))
        self._set_text("right_enable", _format_bool(snapshot.enable_right))

        self._set_text("left_error_norm_mm", _format_float(snapshot.left.error_norm_mm, 2, " mm"))
        self._set_text("right_error_norm_mm", _format_float(snapshot.right.error_norm_mm, 2, " mm"))

        self._set_text("pico_frame_age_ms", _format_float(snapshot.pico_frame_age_ms, 1, " ms"))
        self._set_text("command_loop_dt_ms", _format_float(snapshot.command_loop_dt_ms, 2, " ms"))
        self._set_text("target_age_ms", _format_float(snapshot.target_age_ms, 1, " ms"))

        self._set_text("ik_status", str(snapshot.ik_status) if snapshot.ik_status else "-")
        self._set_text("sdk_status", str(snapshot.sdk_status) if snapshot.sdk_status else "-")
        self._set_text("left_relative_angle_deg", _format_float(snapshot.left_relative_angle_deg, 2, " deg"))
        self._set_text("right_relative_angle_deg", _format_float(snapshot.right_relative_angle_deg, 2, " deg"))

        self._set_text("logging_enabled", _format_bool(snapshot.logging_enabled))
        self._set_text("dropped_log_count", str(int(snapshot.dropped_log_count)))

    def show_waiting(self, text: str = "Waiting for snapshot...") -> None:
        self._set_text("global_status", text)
        self._set_text("safety_state", "UNKNOWN")

    def _set_text(self, key: str, value: str) -> None:
        label = self._labels.get(key)
        if label is not None:
            label.setText(value)


_STATUS_ROWS = [
    ("pico_connected", "Pico connected"),
    ("robot_connected", "Robot connected"),
    ("safety_state", "Safety state"),
    ("global_status", "Global status"),
    ("teleop_mode", "Teleop mode"),
    ("orientation_tracking", "Orientation tracking"),
    ("orientation_relative_mode", "Orientation relative mode"),
    ("left_calibrated", "Left calibrated"),
    ("left_active", "Left active"),
    ("left_enable", "Left enable/deadman"),
    ("right_calibrated", "Right calibrated"),
    ("right_active", "Right active"),
    ("right_enable", "Right enable/deadman"),
    ("left_error_norm_mm", "Left error norm"),
    ("right_error_norm_mm", "Right error norm"),
    ("pico_frame_age_ms", "Pico frame age"),
    ("command_loop_dt_ms", "Command loop dt"),
    ("target_age_ms", "Target age"),
    ("ik_status", "IK status"),
    ("sdk_status", "SDK status"),
    ("left_relative_angle_deg", "Left relative angle"),
    ("right_relative_angle_deg", "Right relative angle"),
    ("logging_enabled", "Logging enabled"),
    ("dropped_log_count", "Dropped log count"),
]


def _format_bool(value: bool) -> str:
    return "YES" if bool(value) else "NO"


def _format_float(value: float | None, digits: int, suffix: str = "") -> str:
    if value is None:
        return "-"
    return f"{float(value):.{int(digits)}f}{suffix}"


__all__ = ["TeleopStatusPanel"]
