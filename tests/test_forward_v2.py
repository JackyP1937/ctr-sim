import numpy as np

from ctr_sim.mechanics.forward_v2 import (
    solve_forward_kinematics_v2,
)


def test_forward_kinematics_v2(forward_robot):

    backbone = solve_forward_kinematics_v2(
        forward_robot,
    )

    #
    # We should have one solved segment
    # for every geometric segment.
    #
    assert len(backbone.segments) > 0

    #
    # Inspect the first solved segment.
    #
    segment = backbone.segments[0]

    assert segment.theta.shape == (
        len(forward_robot.tubes),
    )







    assert segment.curvature.shape == (3,)

    #
    # The segment should contain the dense IVP solution.
    #
    assert segment.ivp_solution is not None

    #
    # The final state should be finite.
    #
    assert np.all(np.isfinite(segment.ivp_solution.y))

    #
    # Dense output should be available.
    #
    assert segment.ivp_solution.sol is not None

