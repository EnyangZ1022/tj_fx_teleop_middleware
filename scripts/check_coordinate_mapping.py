from __future__ import annotations

from pathlib import Path
import sys

# Allow running this script directly without installing the package.
ROOT_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from teleop.diagnostics.integration_checks import check_coordinate_axis_mapping


def _format_vec(v: tuple[float, float, float]) -> str:
    return f"({v[0]:+.1f}, {v[1]:+.1f}, {v[2]:+.1f})"


def main() -> None:
    right = check_coordinate_axis_mapping("right")
    left = check_coordinate_axis_mapping("left")

    print("Stage 6C coordinate mapping check (user frame -> arm SDK delta)")
    print("Assumed user frame from received data: +X right, +Y up, +Z forward/in")
    print("")

    print("Right arm mapping table:")
    print(f"  user +X right    -> robot delta {_format_vec(right['user_+X'])}")
    print(f"  user +Y up       -> robot delta {_format_vec(right['user_+Y'])}")
    print(f"  user +Z forward  -> robot delta {_format_vec(right['user_+Z'])}")
    print("  right: user +Z forward -> robot +X forward")
    print("  right: user +Y up -> robot +Y up")
    print("  right: user +X right -> robot +Z right")
    print("")

    print("Left arm mapping table:")
    print(f"  user +X right    -> robot delta {_format_vec(left['user_+X'])}")
    print(f"  user +Y up       -> robot delta {_format_vec(left['user_+Y'])}")
    print(f"  user +Z forward  -> robot delta {_format_vec(left['user_+Z'])}")
    print("  left: user +Z forward -> robot +X forward")
    print("  left: user +Y up -> robot -Y because left SDK +Y is down")
    print("  left: user +X right -> robot -Z because left SDK +Z is left")


if __name__ == "__main__":
    main()
