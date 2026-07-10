from dataclasses import dataclass

from ctr_sim.tube import Tube


@dataclass
class Segment:
    """
    Represents one contiguous backbone segment.

    Every point within the segment contains the same set of tubes.
    """

    start: float
    end: float
    tubes: list[Tube]