import numpy as np

from .torsion import (
    evaluate_torsion_solution,
    solve_torsion_bvp
)
from .integration import (
    integrate_backbone,
    resultant_curvature
)

from ctr_sim.robot import ConcentricTubeRobot
from ctr_sim.solution import CTRSolution

def solve_forward_kinematics(
    robot: ConcentricTubeRobot,
) -> CTRSolution:
    """
    Solve the unloaded forward kinematics of a concentric tube robot.
    """

    bvp_solution = solve_torsion_bvp(robot)

    ds = 1e-3

    s = np.arange(
        0.0,
        robot.state.insertions[-1] + ds,
        ds,
    )

    theta = evaluate_torsion_solution(
        bvp_solution,
        s,
    )

    u = resultant_curvature(
        robot,
        theta,
        s,
    )

    backbone = integrate_backbone(
        u,
        s,
        robot,
    )

    return CTRSolution(
        robot=robot,
        backbone=backbone,
        torsion=theta,
        curvature=u,
        s=s,
        tip_position=backbone.position[-1],
        tip_rotation=backbone.rotation[-1],
    )