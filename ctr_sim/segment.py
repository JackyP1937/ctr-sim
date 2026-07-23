from dataclasses import dataclass

from ctr_sim.tube import Tube


@dataclass
class Segment:
    """
    A contiguous interval of the backbone over which the set of active tubes
    remains constant.
    """

    start: float
    end: float
    active_tubes: list[Tube]

    