"""Rail-relative measurement and configurable inspection rules."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Measurement:
    height_m: float
    rail_reference_y_m: float
    threshold_m: float | None
    rule: str | None
    status: str
    valid: bool


def measure_height(point_world_y_m: float, rail_reference_y_m: float | None) -> float:
    if rail_reference_y_m is None:
        raise ValueError("rail reference height is required")
    return float(point_world_y_m - rail_reference_y_m)


def evaluate_clearance(
    point_world_y_m: float,
    rail_reference_y_m: float | None,
    *,
    threshold_m: float | None,
    rule: str | None,
) -> Measurement:
    height = measure_height(point_world_y_m, rail_reference_y_m)
    if rule is None or threshold_m is None:
        return Measurement(height, rail_reference_y_m, threshold_m, rule, "RULE_UNCONFIGURED", False)
    if rule == "maximum_height":
        status = "PASS" if height <= threshold_m else "FAIL"
    elif rule == "minimum_clearance":
        status = "PASS" if height >= threshold_m else "FAIL"
    elif rule == "absolute_separation":
        status = "PASS" if abs(height) <= threshold_m else "FAIL"
    else:
        raise ValueError(f"unsupported clearance rule: {rule}")
    return Measurement(height, rail_reference_y_m, threshold_m, rule, status, True)
