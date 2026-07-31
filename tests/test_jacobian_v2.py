import numpy as np

from ctr_sim.control.jacobian import (
    numerical_position_jacobian_v2,
)


def test_position_jacobian_v2_shape(
    forward_robot,
):

    J = numerical_position_jacobian_v2(
        forward_robot,
    )

    assert J.shape == (
        3,
        2 * len(forward_robot.tubes),
    )

    assert np.all(
        np.isfinite(J)
    )

    print()
    print(np.round(J, 3))