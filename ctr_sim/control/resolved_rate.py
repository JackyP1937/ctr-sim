import numpy as np

from ctr_sim.robot import ConcentricTubeRobot
from ctr_sim.control.jacobian import numerical_position_jacobian


def resolved_rate_step(
    robot: ConcentricTubeRobot,
    dx: np.ndarray,
) -> np.ndarray:
    """
    Compute a joint-space increment that best achieves a desired
    Cartesian position increment.

    Parameters
    ----------
    robot : ConcentricTubeRobot
        Current robot configuration.

    dx : np.ndarray
        Desired Cartesian position increment with shape (3,).

    Returns
    -------
    np.ndarray
        Joint increment vector with shape (2 * n_tubes,).

        The entries are ordered as

            [Δβ₁ Δβ₂ ... Δβₙ Δα₁ Δα₂ ... Δαₙ]
    """

    J = numerical_position_jacobian(robot)

    gain = 0.1

    dq = gain * np.linalg.pinv(J) @ dx

    return dq