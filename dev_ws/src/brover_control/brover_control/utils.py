#!/usr/bin/env python3

# _LEFT_SLOPE_DEG_PER_COUNT  = (-18.3) / (-1701.0)   # ≈ 0.010758377425044091
# _RIGHT_SLOPE_DEG_PER_COUNT = ( 11.45) / ( 2140.0)  # ≈ 0.005350467289719626
_LEFT_SLOPE_DEG_PER_COUNT = (-16.0) / (-1593.0)
_RIGHT_SLOPE_DEG_PER_COUNT = (18.0) / (1611.0)

# Safety clamps (mechanical limits in degrees)
# _MIN_DEG = -18.3
# _MAX_DEG =  11.45
_MIN_DEG = -22.5
_MAX_DEG = 22.5


def counts_to_deg(
    counts: int, zero_counts: int = 0, clamp: bool = True
) -> float:
    """
    Convert encoder counts (motor-side) to steering-wheel degrees (gearbox output),
    using a piecewise-linear map around the zero point.
    - counts: current raw encoder counts
    - zero_counts: counts that correspond to 0° (you said you set this to 0 at straight)
    - clamp: if True, limit to your mechanical stop angles
    """
    rel = counts - zero_counts
    slope = (
        _RIGHT_SLOPE_DEG_PER_COUNT if rel >= 0 else _LEFT_SLOPE_DEG_PER_COUNT
    )
    angle = rel * slope
    if clamp:
        if angle < _MIN_DEG:
            angle = _MIN_DEG
        if angle > _MAX_DEG:
            angle = _MAX_DEG
    return angle
