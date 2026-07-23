from dataclasses import dataclass

from ctr_sim.segment import Segment


@dataclass
class Backbone:
    """
    Segment-wise representation of the robot backbone.
    """

    segments: list[Segment]

    