from dataclasses import dataclass

from ctr_sim.tube import Tube
from ctr_sim.state import CTRState


@dataclass
class ConcentricTubeRobot:
    """
    Represents a concentric tube robot.

    Tube Ordering
    -------------
    The tubes must be ordered from outermost to innermost.

    Index 0 : Outermost tube
    Index 1 : Second tube
    ...
    Index N-1 : Innermost tube

    The insertions and rotations stored in the robot state follow the
    same ordering as the tube list.
    """

    tubes: list[Tube]
    state: CTRState

    def __post_init__(self):

        if len(self.tubes) == 0:
            raise ValueError(
                "Robot must contain at least one tube."
            )

        if len(self.tubes) != len(self.state.insertions):
            raise ValueError(
                "Number of insertions must equal number of tubes."
            )

        if len(self.tubes) != len(self.state.rotations):
            raise ValueError(
                "Number of rotations must equal number of tubes."
            )

        # Check tube lengths
        for i in range(len(self.tubes) - 1):

            if self.tubes[i].length >= self.tubes[i + 1].length:
                raise ValueError(
                    "Tube lengths must increase from outer to inner."
                )

        # Check tube diameters
        for i in range(len(self.tubes) - 1):

            if self.tubes[i].inner_diameter <= self.tubes[i + 1].outer_diameter:
                raise ValueError(
                    "Each tube must fit inside the previous tube."
                )