from __future__ import annotations

from dataclasses import dataclass, replace
import math

from teleop.core.pose import Pose7
from teleop.core.teleop_frame import TeleopFrame


_ALLOWED_PICO_RESAMPLE_MODES = {"latest", "predictive"}
_MIN_VALID_VELOCITY_DT_MS = 0.5
_MAX_VALID_VELOCITY_DT_MS = 300.0
_MAX_ABNORMAL_RECEIVER_SEQ_JUMP = 20


Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class PicoResamplerConfig:
    mode: str = "latest"
    extrapolation_horizon_ms: float = 15.0
    prediction_max_frame_age_ms: float = 50.0
    velocity_filter_beta: float = 0.5
    max_predicted_step_mm: float = 5.0

    def __post_init__(self) -> None:
        mode = str(self.mode).strip().lower()
        if mode not in _ALLOWED_PICO_RESAMPLE_MODES:
            raise ValueError(f"mode must be one of {sorted(_ALLOWED_PICO_RESAMPLE_MODES)}")
        object.__setattr__(self, "mode", mode)

        if float(self.extrapolation_horizon_ms) <= 0.0:
            raise ValueError("extrapolation_horizon_ms must be > 0")
        if float(self.prediction_max_frame_age_ms) <= 0.0:
            raise ValueError("prediction_max_frame_age_ms must be > 0")

        beta = float(self.velocity_filter_beta)
        if beta < 0.0 or beta > 1.0:
            raise ValueError("velocity_filter_beta must be within [0, 1]")

        if float(self.max_predicted_step_mm) <= 0.0:
            raise ValueError("max_predicted_step_mm must be > 0")


@dataclass(frozen=True)
class PicoResamplerResult:
    frame: TeleopFrame | None
    mode: str
    prediction_used: bool
    prediction_h_ms: float | None
    prediction_clamped: bool
    prediction_frame_age_ms: float | None
    prediction_reason: str
    latest_left_input_speed_mm_s: float | None
    latest_right_input_speed_mm_s: float | None
    predicted_left_pos_step_mm: float | None
    predicted_right_pos_step_mm: float | None


class PicoCausalResampler:
    """Low-latency causal Pico frame resampler.

    This class intentionally keeps tiny O(1) state and never waits for future frames.
    """

    def __init__(self, config: PicoResamplerConfig | None = None):
        self._config = config if config is not None else PicoResamplerConfig()
        self._previous_actual: TeleopFrame | None = None
        self._latest_actual: TeleopFrame | None = None
        self._last_receiver_seq: int | None = None
        self._left_velocity_m_s: Vec3 | None = None
        self._right_velocity_m_s: Vec3 | None = None
        self._last_teleop_mode: str | None = None

    @property
    def mode(self) -> str:
        return str(self._config.mode)

    def reset(self) -> None:
        self._reset_prediction_state(keep_latest=True)

    def process(
        self,
        *,
        latest_actual_frame: TeleopFrame | None,
        now_ns: int,
        teleop_mode: str | None = None,
    ) -> PicoResamplerResult:
        curr_ns = int(now_ns)

        if teleop_mode is not None:
            curr_mode = str(teleop_mode)
            if self._last_teleop_mode is None:
                self._last_teleop_mode = curr_mode
            elif self._last_teleop_mode != curr_mode:
                self._last_teleop_mode = curr_mode
                self._reset_prediction_state(keep_latest=True)
                self._track_latest_only(latest_actual_frame)
                return self._build_result(
                    frame=latest_actual_frame,
                    prediction_used=False,
                    prediction_h_ms=None,
                    prediction_clamped=False,
                    prediction_frame_age_ms=self._frame_age_ms(curr_ns, latest_actual_frame),
                    prediction_reason="reset",
                    predicted_left_pos_step_mm=None,
                    predicted_right_pos_step_mm=None,
                )

        if self.mode == "latest":
            self._track_latest_only(latest_actual_frame)
            return self._build_result(
                frame=latest_actual_frame,
                prediction_used=False,
                prediction_h_ms=None,
                prediction_clamped=False,
                prediction_frame_age_ms=self._frame_age_ms(curr_ns, latest_actual_frame),
                prediction_reason="latest_mode",
                predicted_left_pos_step_mm=None,
                predicted_right_pos_step_mm=None,
            )

        if self.mode != "predictive":
            self._track_latest_only(latest_actual_frame)
            return self._build_result(
                frame=latest_actual_frame,
                prediction_used=False,
                prediction_h_ms=None,
                prediction_clamped=False,
                prediction_frame_age_ms=self._frame_age_ms(curr_ns, latest_actual_frame),
                prediction_reason="disabled",
                predicted_left_pos_step_mm=None,
                predicted_right_pos_step_mm=None,
            )

        if latest_actual_frame is None:
            self._reset_prediction_state(keep_latest=False)
            return self._build_result(
                frame=None,
                prediction_used=False,
                prediction_h_ms=None,
                prediction_clamped=False,
                prediction_frame_age_ms=None,
                prediction_reason="no_previous_frame",
                predicted_left_pos_step_mm=None,
                predicted_right_pos_step_mm=None,
            )

        if self._is_new_actual_frame(latest_actual_frame):
            prediction_reason = self._ingest_new_actual_frame(latest_actual_frame)
            return self._build_result(
                frame=latest_actual_frame,
                prediction_used=False,
                prediction_h_ms=None,
                prediction_clamped=False,
                prediction_frame_age_ms=self._frame_age_ms(curr_ns, latest_actual_frame),
                prediction_reason=prediction_reason,
                predicted_left_pos_step_mm=None,
                predicted_right_pos_step_mm=None,
            )

        return self._predict_without_new_frame(curr_ns)

    def _predict_without_new_frame(self, now_ns: int) -> PicoResamplerResult:
        latest_actual = self._latest_actual
        if latest_actual is None:
            return self._build_result(
                frame=None,
                prediction_used=False,
                prediction_h_ms=None,
                prediction_clamped=False,
                prediction_frame_age_ms=None,
                prediction_reason="no_previous_frame",
                predicted_left_pos_step_mm=None,
                predicted_right_pos_step_mm=None,
            )

        frame_age_ms = self._frame_age_ms(now_ns, latest_actual)
        if frame_age_ms is None:
            return self._build_result(
                frame=latest_actual,
                prediction_used=False,
                prediction_h_ms=None,
                prediction_clamped=False,
                prediction_frame_age_ms=None,
                prediction_reason="no_previous_frame",
                predicted_left_pos_step_mm=None,
                predicted_right_pos_step_mm=None,
            )

        if frame_age_ms > float(self._config.prediction_max_frame_age_ms):
            self._reset_prediction_state(keep_latest=True)
            return self._build_result(
                frame=latest_actual,
                prediction_used=False,
                prediction_h_ms=None,
                prediction_clamped=False,
                prediction_frame_age_ms=frame_age_ms,
                prediction_reason="frame_too_old",
                predicted_left_pos_step_mm=None,
                predicted_right_pos_step_mm=None,
            )

        if self._previous_actual is None:
            return self._build_result(
                frame=latest_actual,
                prediction_used=False,
                prediction_h_ms=None,
                prediction_clamped=False,
                prediction_frame_age_ms=frame_age_ms,
                prediction_reason="no_previous_frame",
                predicted_left_pos_step_mm=None,
                predicted_right_pos_step_mm=None,
            )

        h_ms = min(frame_age_ms, float(self._config.extrapolation_horizon_ms))
        h_s = max(0.0, h_ms / 1000.0)

        left_pose, left_step_mm, left_clamped, left_used = self._predict_pose(
            pose=latest_actual.left.pose_pico,
            velocity_m_s=self._left_velocity_m_s,
            horizon_s=h_s,
        )
        right_pose, right_step_mm, right_clamped, right_used = self._predict_pose(
            pose=latest_actual.right.pose_pico,
            velocity_m_s=self._right_velocity_m_s,
            horizon_s=h_s,
        )

        prediction_used = bool(left_used or right_used)
        if not prediction_used:
            return self._build_result(
                frame=latest_actual,
                prediction_used=False,
                prediction_h_ms=None,
                prediction_clamped=False,
                prediction_frame_age_ms=frame_age_ms,
                prediction_reason="no_previous_frame",
                predicted_left_pos_step_mm=None,
                predicted_right_pos_step_mm=None,
            )

        predicted_frame = latest_actual
        if left_pose is not None:
            predicted_left = replace(predicted_frame.left, pose_pico=left_pose)
            predicted_frame = replace(predicted_frame, left=predicted_left)
        if right_pose is not None:
            predicted_right = replace(predicted_frame.right, pose_pico=right_pose)
            predicted_frame = replace(predicted_frame, right=predicted_right)

        prediction_clamped = bool(left_clamped or right_clamped)
        prediction_reason = "predicted"
        if frame_age_ms > float(self._config.extrapolation_horizon_ms):
            prediction_reason = "horizon_clamped"
        if prediction_clamped:
            prediction_reason = "step_clamped"

        return self._build_result(
            frame=predicted_frame,
            prediction_used=True,
            prediction_h_ms=h_ms,
            prediction_clamped=prediction_clamped,
            prediction_frame_age_ms=frame_age_ms,
            prediction_reason=prediction_reason,
            predicted_left_pos_step_mm=left_step_mm,
            predicted_right_pos_step_mm=right_step_mm,
        )

    def _ingest_new_actual_frame(self, frame: TeleopFrame) -> str:
        previous_actual = self._latest_actual
        receiver_seq = self._extract_receiver_seq(frame)

        if receiver_seq is not None and self._last_receiver_seq is not None:
            seq_delta = receiver_seq - self._last_receiver_seq
            if abs(seq_delta) > _MAX_ABNORMAL_RECEIVER_SEQ_JUMP:
                self._latest_actual = frame
                self._last_receiver_seq = receiver_seq
                self._reset_prediction_state(keep_latest=True)
                return "reset"

        self._latest_actual = frame
        self._last_receiver_seq = receiver_seq if receiver_seq is not None else self._last_receiver_seq

        if previous_actual is None:
            self._previous_actual = None
            self._left_velocity_m_s = None
            self._right_velocity_m_s = None
            return "new_frame"

        dt_s = self._compute_velocity_dt_s(previous_actual, frame)
        if dt_s is None:
            self._reset_prediction_state(keep_latest=True)
            return "invalid_dt"

        left_velocity = self._estimate_velocity(
            previous_pose=previous_actual.left.pose_pico,
            latest_pose=frame.left.pose_pico,
            dt_s=dt_s,
            previous_velocity=self._left_velocity_m_s,
        )
        right_velocity = self._estimate_velocity(
            previous_pose=previous_actual.right.pose_pico,
            latest_pose=frame.right.pose_pico,
            dt_s=dt_s,
            previous_velocity=self._right_velocity_m_s,
        )

        if left_velocity is None and right_velocity is None:
            self._reset_prediction_state(keep_latest=True)
            return "invalid_dt"

        self._previous_actual = previous_actual
        self._left_velocity_m_s = left_velocity
        self._right_velocity_m_s = right_velocity
        return "new_frame"

    def _track_latest_only(self, frame: TeleopFrame | None) -> None:
        if frame is None:
            self._reset_prediction_state(keep_latest=False)
            return

        if self._is_new_actual_frame(frame):
            self._latest_actual = frame
            receiver_seq = self._extract_receiver_seq(frame)
            self._last_receiver_seq = receiver_seq if receiver_seq is not None else self._last_receiver_seq

    def _is_new_actual_frame(self, frame: TeleopFrame) -> bool:
        if self._latest_actual is None:
            return True

        curr_receiver_seq = self._extract_receiver_seq(frame)
        last_receiver_seq = self._extract_receiver_seq(self._latest_actual)
        if curr_receiver_seq is not None and last_receiver_seq is not None:
            return curr_receiver_seq != last_receiver_seq

        return int(frame.frame_id) != int(self._latest_actual.frame_id)

    def _reset_prediction_state(self, *, keep_latest: bool) -> None:
        self._previous_actual = None
        self._left_velocity_m_s = None
        self._right_velocity_m_s = None

        if not keep_latest:
            self._latest_actual = None
            self._last_receiver_seq = None

    def _compute_velocity_dt_s(self, previous: TeleopFrame, latest: TeleopFrame) -> float | None:
        source_dt_ns = int(latest.source_timestamp_ns) - int(previous.source_timestamp_ns)
        if self._is_valid_dt_ns(source_dt_ns):
            return float(source_dt_ns) / 1_000_000_000.0

        pc_dt_ns = int(latest.pc_receive_time_ns) - int(previous.pc_receive_time_ns)
        if self._is_valid_dt_ns(pc_dt_ns):
            return float(pc_dt_ns) / 1_000_000_000.0

        return None

    @staticmethod
    def _is_valid_dt_ns(dt_ns: int) -> bool:
        if dt_ns <= 0:
            return False
        dt_ms = float(dt_ns) / 1_000_000.0
        if not math.isfinite(dt_ms):
            return False
        if dt_ms < _MIN_VALID_VELOCITY_DT_MS:
            return False
        if dt_ms > _MAX_VALID_VELOCITY_DT_MS:
            return False
        return True

    def _estimate_velocity(
        self,
        *,
        previous_pose: Pose7 | None,
        latest_pose: Pose7 | None,
        dt_s: float,
        previous_velocity: Vec3 | None,
    ) -> Vec3 | None:
        if previous_pose is None or latest_pose is None:
            return None
        if not self._is_finite_xyz(previous_pose) or not self._is_finite_xyz(latest_pose):
            return None
        if dt_s <= 0.0:
            return None

        vx_new = (float(latest_pose.x) - float(previous_pose.x)) / dt_s
        vy_new = (float(latest_pose.y) - float(previous_pose.y)) / dt_s
        vz_new = (float(latest_pose.z) - float(previous_pose.z)) / dt_s
        if not self._is_finite_vec((vx_new, vy_new, vz_new)):
            return None

        if previous_velocity is None:
            return (vx_new, vy_new, vz_new)

        beta = float(self._config.velocity_filter_beta)
        return (
            beta * vx_new + (1.0 - beta) * float(previous_velocity[0]),
            beta * vy_new + (1.0 - beta) * float(previous_velocity[1]),
            beta * vz_new + (1.0 - beta) * float(previous_velocity[2]),
        )

    def _predict_pose(
        self,
        *,
        pose: Pose7 | None,
        velocity_m_s: Vec3 | None,
        horizon_s: float,
    ) -> tuple[Pose7 | None, float | None, bool, bool]:
        if pose is None or velocity_m_s is None:
            return None, None, False, False
        if not self._is_finite_xyz(pose) or not self._is_finite_vec(velocity_m_s):
            return None, None, False, False

        dx = float(velocity_m_s[0]) * float(horizon_s)
        dy = float(velocity_m_s[1]) * float(horizon_s)
        dz = float(velocity_m_s[2]) * float(horizon_s)
        if not self._is_finite_vec((dx, dy, dz)):
            return None, None, False, False

        step_m = self._norm((dx, dy, dz))
        max_step_m = float(self._config.max_predicted_step_mm) / 1000.0
        clamped = False
        if step_m > max_step_m and step_m > 0.0:
            scale = max_step_m / step_m
            dx *= scale
            dy *= scale
            dz *= scale
            step_m = max_step_m
            clamped = True

        predicted = Pose7(
            x=float(pose.x) + dx,
            y=float(pose.y) + dy,
            z=float(pose.z) + dz,
            qx=float(pose.qx),
            qy=float(pose.qy),
            qz=float(pose.qz),
            qw=float(pose.qw),
        )
        if not self._is_finite_xyz(predicted):
            return None, None, False, False

        return predicted, float(step_m) * 1000.0, clamped, True

    @staticmethod
    def _frame_age_ms(now_ns: int, frame: TeleopFrame | None) -> float | None:
        if frame is None:
            return None
        pc_receive_ns = int(frame.pc_receive_time_ns)
        if pc_receive_ns <= 0:
            return None
        return max(0.0, float(int(now_ns) - pc_receive_ns) / 1_000_000.0)

    @staticmethod
    def _extract_receiver_seq(frame: TeleopFrame | None) -> int | None:
        if frame is None or frame.receiver_seq is None:
            return None
        return int(frame.receiver_seq)

    @staticmethod
    def _norm(vec: Vec3) -> float:
        return math.sqrt(float(vec[0]) ** 2 + float(vec[1]) ** 2 + float(vec[2]) ** 2)

    @staticmethod
    def _is_finite_vec(vec: Vec3) -> bool:
        return all(math.isfinite(float(value)) for value in vec)

    @staticmethod
    def _is_finite_xyz(pose: Pose7) -> bool:
        return math.isfinite(float(pose.x)) and math.isfinite(float(pose.y)) and math.isfinite(float(pose.z))

    def _build_result(
        self,
        *,
        frame: TeleopFrame | None,
        prediction_used: bool,
        prediction_h_ms: float | None,
        prediction_clamped: bool,
        prediction_frame_age_ms: float | None,
        prediction_reason: str,
        predicted_left_pos_step_mm: float | None,
        predicted_right_pos_step_mm: float | None,
    ) -> PicoResamplerResult:
        return PicoResamplerResult(
            frame=frame,
            mode=self.mode,
            prediction_used=bool(prediction_used),
            prediction_h_ms=float(prediction_h_ms) if prediction_h_ms is not None else None,
            prediction_clamped=bool(prediction_clamped),
            prediction_frame_age_ms=(
                float(prediction_frame_age_ms) if prediction_frame_age_ms is not None else None
            ),
            prediction_reason=str(prediction_reason),
            latest_left_input_speed_mm_s=self._speed_mm_s(self._left_velocity_m_s),
            latest_right_input_speed_mm_s=self._speed_mm_s(self._right_velocity_m_s),
            predicted_left_pos_step_mm=(
                float(predicted_left_pos_step_mm) if predicted_left_pos_step_mm is not None else None
            ),
            predicted_right_pos_step_mm=(
                float(predicted_right_pos_step_mm) if predicted_right_pos_step_mm is not None else None
            ),
        )

    @staticmethod
    def _speed_mm_s(velocity_m_s: Vec3 | None) -> float | None:
        if velocity_m_s is None:
            return None
        speed_m_s = math.sqrt(
            float(velocity_m_s[0]) ** 2 + float(velocity_m_s[1]) ** 2 + float(velocity_m_s[2]) ** 2
        )
        if not math.isfinite(speed_m_s):
            return None
        return speed_m_s * 1000.0


__all__ = ["PicoResamplerConfig", "PicoResamplerResult", "PicoCausalResampler"]
