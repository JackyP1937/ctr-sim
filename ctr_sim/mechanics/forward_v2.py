"""
Segment-aware forward mechanics solver.

This module implements the next-generation forward mechanics solver
using a segment-wise backbone representation.
"""

import numpy as np

from ctr_sim.robot import ConcentricTubeRobot
from ctr_sim.backbone import Backbone
from ctr_sim.segment_solution import SegmentSolution

# from .torsion import (
#     solve_torsion_bvp,
#     evaluate_segment_torsion,
# )

from .torsion_v2 import (
    solve_torsion_v2,
    evaluate_segment_torsion_v2,
)

# from .curvature import (
#     segment_curvature,
# )

from .curvature import (
    segment_curvature_v2,
)

from .integration import (
    integrate_segment,
)

from ctr_sim.kinematics.intervals import (
    backbone_segments,
)


def solve_forward_kinematics_v2(
    robot: ConcentricTubeRobot,
    torsion_initial_guess: np.ndarray | None = None,
) -> Backbone:
    """
    Solve the unloaded forward mechanics using the
    segment-aware mechanics pipeline.

    Parameters
    ----------
    robot : ConcentricTubeRobot
        Robot configuration.

    torsion_initial_guess : np.ndarray, optional
        Initial guess for the base torsional strains
        theta_dot_i(0).
    """

    #
    # Step 1
    # Partition the robot.
    #
    segments = backbone_segments(robot)

    #
    # Step 2
    # Solve torsion.
    #

    # bvp_solution = solve_torsion_bvp(robot)
    torsion_solution = solve_torsion_v2(
        robot,
        initial_guess=torsion_initial_guess,
    )

    #
    # Step 3
    # Integrate segment by segment.
    #
    position = np.zeros(3)
    rotation = np.eye(3)

    segment_solutions = []

    for segment in segments:

        # theta = evaluate_segment_torsion(
        #     robot,
        #     bvp_solution,
        #     segment,
        # )

        theta = evaluate_segment_torsion_v2(
            torsion_solution,
            segment,
        )

        # curvature = segment_curvature(
        #     robot,
        #     theta,
        #     segment,
        # )

        curvature = segment_curvature_v2(
            theta,
            segment,
        )

        ivp_solution, position1, rotation1 = integrate_segment(
            position,
            rotation,
            curvature,
            segment,
        )

        segment_solutions.append(
            SegmentSolution(
                segment=segment,
                theta=theta,
                curvature=curvature,
                ivp_solution=ivp_solution,
            )
        )

        position = position1
        rotation = rotation1

    return Backbone(
        segments=segment_solutions,
        torsion_solution=torsion_solution,
    )

    