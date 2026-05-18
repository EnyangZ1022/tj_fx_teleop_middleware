from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable


@dataclass(frozen=True)
class Pose7:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float

    @classmethod
    def from_tuple(cls, values: Iterable[float]) -> "Pose7":
        values_tuple = tuple(float(v) for v in values)
        if len(values_tuple) != 7:
            raise ValueError("Pose7 requires exactly 7 values")
        return cls(*values_tuple)

    def as_tuple(self) -> tuple[float, float, float, float, float, float, float]:
        return (self.x, self.y, self.z, self.qx, self.qy, self.qz, self.qw)

    def is_finite(self) -> bool:
        return all(math.isfinite(v) for v in self.as_tuple())

    def quaternion_norm(self) -> float:
        return math.sqrt(self.qx * self.qx + self.qy * self.qy + self.qz * self.qz + self.qw * self.qw)

    def is_zero_pose(self) -> bool:
        eps = 1e-9
        return all(abs(v) <= eps for v in self.as_tuple())

    def is_valid(self) -> bool:
        if not self.is_finite():
            return False
        if self.is_zero_pose():
            return False
        return self.quaternion_norm() > 1e-6
