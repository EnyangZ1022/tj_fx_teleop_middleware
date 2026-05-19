from __future__ import annotations

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QHBoxLayout, QLabel, QMainWindow, QWidget

from teleop.ui.scene3d import TeleopScene3DWidget
from teleop.ui.snapshot import LatestSnapshotStore
from teleop.ui.status_panel import TeleopStatusPanel
from teleop.ui.ui_config import UIConfig


class TeleopDiagnosticWindow(QMainWindow):
    """Main Stage 8 diagnostics window.

    The window only reads latest snapshots and never owns command/safety logic.
    """

    def __init__(self, snapshot_store: LatestSnapshotStore, config: UIConfig | None = None):
        super().__init__()

        self._snapshot_store = snapshot_store
        self._config = config if config is not None else UIConfig()

        self._scene_widget: TeleopScene3DWidget | None = None
        self._status_panel: TeleopStatusPanel | None = None
        self._waiting_shown = False

        self.setWindowTitle(self._config.window_title)
        self.resize(1280, 760)

        root = QWidget(self)
        layout = QHBoxLayout(root)
        self.setCentralWidget(root)

        if bool(self._config.show_3d_view):
            self._scene_widget = TeleopScene3DWidget(config=self._config, parent=root)
            layout.addWidget(self._scene_widget, stretch=3)
        else:
            placeholder = QLabel("3D view disabled by UI config.")
            layout.addWidget(placeholder, stretch=2)

        if bool(self._config.show_status_panel):
            self._status_panel = TeleopStatusPanel(parent=root)
            layout.addWidget(self._status_panel, stretch=2)

        self._timer = QTimer(self)
        self._timer.setInterval(int(self._config.timer_interval_ms()))
        self._timer.timeout.connect(self._on_timer_tick)
        self._timer.start()

    def _on_timer_tick(self) -> None:
        snapshot = self._snapshot_store.get()
        if snapshot is None:
            self._show_waiting_state()
            return

        self._waiting_shown = False
        if self._scene_widget is not None:
            self._scene_widget.update_from_snapshot(snapshot)
        if self._status_panel is not None:
            self._status_panel.update_from_snapshot(snapshot)

    def _show_waiting_state(self) -> None:
        if self._waiting_shown:
            return

        if self._status_panel is not None:
            self._status_panel.show_waiting()
        self._waiting_shown = True


__all__ = ["TeleopDiagnosticWindow"]
