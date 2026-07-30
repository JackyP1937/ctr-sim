from dataclasses import dataclass

from ctr_sim.segment_solution import SegmentSolution


@dataclass
class Backbone:
    """
    Segment-wise representation of the robot backbone.
    """

    segments: list[SegmentSolution]

    