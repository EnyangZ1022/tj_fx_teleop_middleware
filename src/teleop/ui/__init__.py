from teleop.ui.snapshot import (
	ArmVisualizationSnapshot,
	LatestSnapshotStore,
	TeleopVisualizationSnapshot,
	compute_error_norm_mm,
)
from teleop.ui.snapshot_builder import (
	build_arm_visualization_snapshot,
	build_visualization_snapshot,
)
from teleop.ui.ui_config import UIConfig

__all__ = [
	"UIConfig",
	"ArmVisualizationSnapshot",
	"TeleopVisualizationSnapshot",
	"LatestSnapshotStore",
	"compute_error_norm_mm",
	"build_arm_visualization_snapshot",
	"build_visualization_snapshot",
]

try:
	from teleop.ui.app import run_ui
	from teleop.ui.main_window import TeleopDiagnosticWindow
	from teleop.ui.scene3d import TeleopScene3DWidget
	from teleop.ui.status_panel import TeleopStatusPanel
except Exception:
	# Keep base UI data models importable in non-GUI environments.
	pass
else:
	__all__.extend(
		[
			"run_ui",
			"TeleopDiagnosticWindow",
			"TeleopScene3DWidget",
			"TeleopStatusPanel",
		]
	)
