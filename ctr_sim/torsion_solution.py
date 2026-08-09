from dataclasses import dataclass

import numpy as np

from ctr_sim.torsion_segment_solution import (
    TorsionSegmentSolution,
)


@dataclass
class TorsionSolution:
    """
    Piecewise segment-aware torsion solution.
    """

    base_theta_dot: np.ndarray

    segments: list[
        TorsionSegmentSolution
    ]