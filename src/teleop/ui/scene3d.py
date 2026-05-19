from __future__ import annotations

import numpy as np
from pyqtgraph.opengl import (
    GLGridItem,
    GLLinePlotItem,
    GLScatterPlotItem,
    GLViewWidget,
)

from teleop.ui.snapshot import ArmVisualizationSnapshot, TeleopVisualizationSnapshot
from teleop.ui.ui_config import UIConfig


class TeleopScene3DWidget(GLViewWidget):
    """Interactive 3D diagnostic scene for dual-arm target/feedback visualization."""

    def __init__(self, config: UIConfig | None = None, parent=None):
        super().__init__(parent=parent)
        self._config = config if config is not None else UIConfig()

        self._left_target_color = np.array([1.0, 0.35, 0.25, 0.95], dtype=float)
        self._left_feedback_color = np.array([0.60, 0.05, 0.05, 0.95], dtype=float)
        self._right_target_color = np.array([0.20, 0.65, 1.0, 0.95], dtype=float)
        self._right_feedback_color = np.array([0.00, 0.95, 0.85, 0.95], dtype=float)

        self._left_line_color = np.array([1.0, 0.7, 0.25, 0.95], dtype=float)
        self._right_line_color = np.array([0.5, 1.0, 0.6, 0.95], dtype=float)

        self._dim_alpha = 0.12

        axis = float(self._config.axis_length_mm)
        self.setCameraPosition(distance=max(250.0, axis * 4.0), elevation=18.0, azimuth=35.0)
        self.setBackgroundColor((16, 18, 22))

        self._grid = GLGridItem()
        self._grid.setSize(x=axis * 2.0, y=axis * 2.0, z=axis * 2.0)
        spacing = max(10.0, axis / 10.0)
        self._grid.setSpacing(x=spacing, y=spacing, z=spacing)
        self.addItem(self._grid)

        self._x_axis = self._make_axis_line((1.0, 0.35, 0.35, 0.9), axis, (1.0, 0.0, 0.0))
        self._y_axis = self._make_axis_line((0.35, 1.0, 0.45, 0.9), axis, (0.0, 1.0, 0.0))
        self._z_axis = self._make_axis_line((0.35, 0.6, 1.0, 0.9), axis, (0.0, 0.0, 1.0))

        self.addItem(self._x_axis)
        self.addItem(self._y_axis)
        self.addItem(self._z_axis)

        self._left_target_item = self._make_ball_item(self._left_target_color)
        self._left_feedback_item = self._make_ball_item(self._left_feedback_color)
        self._right_target_item = self._make_ball_item(self._right_target_color)
        self._right_feedback_item = self._make_ball_item(self._right_feedback_color)

        self.addItem(self._left_target_item)
        self.addItem(self._left_feedback_item)
        self.addItem(self._right_target_item)
        self.addItem(self._right_feedback_item)

        self._left_error_line = self._make_line_item(self._left_line_color)
        self._right_error_line = self._make_line_item(self._right_line_color)
        self.addItem(self._left_error_line)
        self.addItem(self._right_error_line)

    def update_from_snapshot(self, snapshot: TeleopVisualizationSnapshot) -> None:
        self._update_arm(
            arm=snapshot.left,
            target_item=self._left_target_item,
            feedback_item=self._left_feedback_item,
            error_line_item=self._left_error_line,
            target_color=self._left_target_color,
            feedback_color=self._left_feedback_color,
            line_color=self._left_line_color,
        )
        self._update_arm(
            arm=snapshot.right,
            target_item=self._right_target_item,
            feedback_item=self._right_feedback_item,
            error_line_item=self._right_error_line,
            target_color=self._right_target_color,
            feedback_color=self._right_feedback_color,
            line_color=self._right_line_color,
        )

    def _update_arm(
        self,
        *,
        arm: ArmVisualizationSnapshot,
        target_item: GLScatterPlotItem,
        feedback_item: GLScatterPlotItem,
        error_line_item: GLLinePlotItem,
        target_color: np.ndarray,
        feedback_color: np.ndarray,
        line_color: np.ndarray,
    ) -> None:
        target_visible = arm.target_valid and arm.target_xyz_mm is not None
        feedback_visible = arm.feedback_valid and arm.feedback_xyz_mm is not None

        if target_visible:
            self._show_ball(target_item, arm.target_xyz_mm, target_color)
        else:
            self._dim_ball(target_item, target_color)

        if feedback_visible:
            self._show_ball(feedback_item, arm.feedback_xyz_mm, feedback_color)
        else:
            self._dim_ball(feedback_item, feedback_color)

        show_error_line = bool(self._config.show_error_lines) and target_visible and feedback_visible
        if show_error_line:
            line_pos = np.array([arm.target_xyz_mm, arm.feedback_xyz_mm], dtype=float)
            line_colors = np.array([line_color, line_color], dtype=float)
            error_line_item.setData(pos=line_pos, color=line_colors, width=2.0, antialias=True)
        else:
            self._hide_line(error_line_item, line_color)

    def _show_ball(self, item: GLScatterPlotItem, xyz_mm: tuple[float, float, float], color: np.ndarray) -> None:
        item.setData(
            pos=np.array([[float(xyz_mm[0]), float(xyz_mm[1]), float(xyz_mm[2])]], dtype=float),
            size=float(self._config.ball_size),
            color=np.array([color], dtype=float),
            pxMode=False,
        )

    def _dim_ball(self, item: GLScatterPlotItem, color: np.ndarray) -> None:
        dim = np.array([color[0], color[1], color[2], self._dim_alpha], dtype=float)
        item.setData(
            pos=np.array([[0.0, 0.0, 0.0]], dtype=float),
            size=float(self._config.ball_size) * 0.6,
            color=np.array([dim], dtype=float),
            pxMode=False,
        )

    @staticmethod
    def _hide_line(item: GLLinePlotItem, color: np.ndarray) -> None:
        dim = np.array([color[0], color[1], color[2], 0.0], dtype=float)
        zeros = np.zeros((2, 3), dtype=float)
        item.setData(pos=zeros, color=np.array([dim, dim], dtype=float), width=1.0, antialias=True)

    @staticmethod
    def _make_axis_line(
        color: tuple[float, float, float, float],
        axis_length_mm: float,
        direction: tuple[float, float, float],
    ) -> GLLinePlotItem:
        start = np.array([0.0, 0.0, 0.0], dtype=float)
        end = np.array(
            [
                float(direction[0]) * axis_length_mm,
                float(direction[1]) * axis_length_mm,
                float(direction[2]) * axis_length_mm,
            ],
            dtype=float,
        )
        points = np.array([start, end], dtype=float)
        colors = np.array([color, color], dtype=float)
        return GLLinePlotItem(pos=points, color=colors, width=2.0, antialias=True)

    def _make_ball_item(self, color: np.ndarray) -> GLScatterPlotItem:
        return GLScatterPlotItem(
            pos=np.array([[0.0, 0.0, 0.0]], dtype=float),
            size=float(self._config.ball_size),
            color=np.array([color], dtype=float),
            pxMode=False,
        )

    @staticmethod
    def _make_line_item(color: np.ndarray) -> GLLinePlotItem:
        points = np.zeros((2, 3), dtype=float)
        colors = np.array([color, color], dtype=float)
        return GLLinePlotItem(pos=points, color=colors, width=2.0, antialias=True)


__all__ = ["TeleopScene3DWidget"]
