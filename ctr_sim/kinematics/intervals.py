from ctr_sim.robot import ConcentricTubeRobot
from ctr_sim.segment import Segment
from ctr_sim.types import Interval


def occupied_intervals(
    robot: ConcentricTubeRobot,
) -> list[Interval]:
    """
    Compute the occupied interval of each tube.

    Each interval is returned as

        (proximal_end, distal_tip)

    where

        proximal_end = βᵢ - Lᵢ

        distal_tip = βᵢ

    Parameters
    ----------
    robot : ConcentricTubeRobot
        Robot whose tube intervals are to be computed.

    Returns
    -------
    list[tuple[float, float]]
        List of (proximal_end, distal_tip) intervals.
    """

    intervals = []

    for tube, beta in zip(robot.tubes, robot.state.insertions):

        proximal = beta - tube.length
        distal = beta

        intervals.append((proximal, distal))

    return intervals


def backbone_segments(
    robot: ConcentricTubeRobot,
) -> list[Segment]:
    """
    Partition the robot backbone into contiguous segments.

    Within each segment, the set of active tubes remains constant.
    """

    intervals = occupied_intervals(robot)

    # Collect all interval endpoints
    boundaries = set()

    for start, end in intervals:
        boundaries.add(start)
        boundaries.add(end)

    boundaries = sorted(boundaries)

    segments = []

    # Build each backbone segment
    for i in range(len(boundaries) - 1):

        start = boundaries[i]
        end = boundaries[i + 1]

        midpoint = (start + end) / 2

        active_tubes = []

        for tube, (tube_start, tube_end) in zip(
            robot.tubes,
            intervals,
        ):

            if tube_start <= midpoint <= tube_end:
                active_tubes.append(tube)

        segments.append(
            Segment(
                start=start,
                end=end,
                tubes=active_tubes,
            )
        )

    return segments
