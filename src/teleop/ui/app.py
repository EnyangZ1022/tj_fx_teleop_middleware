from __future__ import annotations

import sys

from PySide6.QtWidgets import QApplication

from teleop.ui.main_window import TeleopDiagnosticWindow
from teleop.ui.snapshot import LatestSnapshotStore
from teleop.ui.ui_config import UIConfig


def run_ui(snapshot_store: LatestSnapshotStore, config: UIConfig | None = None) -> int:
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)

    window = TeleopDiagnosticWindow(snapshot_store=snapshot_store, config=config)
    window.show()
    return int(app.exec())


__all__ = ["run_ui"]
