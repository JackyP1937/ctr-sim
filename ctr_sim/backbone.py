from dataclasses import dataclass

from ctr_sim.segment_solution import SegmentSolution


@dataclass
class Backbone:
    """
    Segment-wise mechanics solution.
    """

    segments: list[SegmentSolution]