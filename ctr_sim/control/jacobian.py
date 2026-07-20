from copy import deepcopy

import numpy as np

from ctr_sim.robot import ConcentricTubeRobot
from ctr_sim.mechanics import solve_forward_kinematics


def numerical_position_jacobian(
    robot: ConcentricTubeRobot,
    delta_insertion: float = 1e-4,
    delta_rotation: float = 1e-4,
) -> np.ndarray:
    """
    Compute the numerical position Jacobian using forward finite differences.

    Parameters
    ----------
    robot : ConcentricTubeRobot
        Robot configuration about which the Jacobian is evaluated.
    delta_insertion : float, optional
        Perturbation applied to tube insertions (m).
    delta_rotation : float, optional
        Perturbation applied to tube rotations (rad).

    Returns
    -------
    np.ndarray
        Position Jacobian with shape (3, 2 * n_tubes).

        The columns are ordered as

            [β₁ β₂ ... βₙ α₁ α₂ ... αₙ]
    """

    n = len(robot.tubes)

    J = np.zeros((3, 2 * n))

    # Nominal tip position
    x0 = solve_forward_kinematics(
        robot,
        ds=1e-4,
    ).tip_position


    #
    # TODO:
    # Insertion derivatives currently depend on the sampled backbone
    # representation used by the forward solver. A future segment-aware
    # mechanics implementation should improve these derivatives.
    #

    #
    # Insertion columns
    #
    for i in range(n):

        robot_plus = deepcopy(robot)

        robot_plus.state.insertions[i] += delta_insertion

        x_plus = solve_forward_kinematics(
            robot_plus,
            ds=1e-4,
        ).tip_position

        J[:, i] = (x_plus - x0) / delta_insertion

    #
    # Rotation columns
    #
    for i in range(n):

        robot_plus = deepcopy(robot)

        robot_plus.state.rotations[i] += delta_rotation

        x_plus = solve_forward_kinematics(
            robot_plus,
            ds=1e-4,
        ).tip_position

        J[:, n + i] = (x_plus - x0) / delta_rotation

    return J