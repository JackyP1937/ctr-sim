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

    theta = evaluate_torsion_solution(
        bvp_solution,
        bvp_solution.x,
    )

    u = resultant_curvature(
        robot,
        theta,
        bvp_solution.x,
    )

    backbone = integrate_backbone(
        u,
        bvp_solution.x,
        robot,
    )

    return CTRSolution(
        backbone=backbone,
        torsion=theta,
        tip_position=backbone.position[-1],
        tip_rotation=backbone.rotation[-1],
    )