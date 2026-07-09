from dataclasses import dataclass


@dataclass
class CTRState:
    """
    Represents the current configuration of a concentric tube robot.

    The robot state contains only the variables that change during operation.

    The insertion and rotation lists correspond directly to the ordering of
    the tubes stored in the associated ConcentricTubeRobot object.

    This class contains no geometric or material information.
    """

    insertions: list[float]

    rotations: list[float]

    