from __future__ import annotations


Vec3 = tuple[float, float, float]


def meters_to_mm(x: float) -> float:
    return float(x) * 1000.0


def mm_to_meters(x: float) -> float:
    return float(x) / 1000.0


def position_m_to_mm(pos: Vec3) -> Vec3:
    return (
        meters_to_mm(pos[0]),
        meters_to_mm(pos[1]),
        meters_to_mm(pos[2]),
    )


def position_mm_to_m(pos: Vec3) -> Vec3:
    return (
        mm_to_meters(pos[0]),
        mm_to_meters(pos[1]),
        mm_to_meters(pos[2]),
    )


__all__ = [
    "meters_to_mm",
    "mm_to_meters",
    "position_m_to_mm",
    "position_mm_to_m",
]
