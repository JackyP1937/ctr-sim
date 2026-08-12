#!/usr/bin/env python3

import numpy as np

import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist

from ctr_sim import (
    Material,
    Tube,
    CTRState,
    ConcentricTubeRobot,
)

from ctr_sim.mechanics.forward_v2 import (
    solve_forward_kinematics_v2,
)

from ctr_sim.mechanics.sampling import (
    sample_backbone,
)

from ctr_sim.control.resolved_rate import (
    resolved_rate_step_v2,
)

from ctr_sim.control.constraints import (
    constrain_joint_step,
)


class CTRSimulator(Node):

    def __init__(self):

        super().__init__(
            "ctr_simulator"
        )

        #
        # Control settings.
        #
        self.control_rate = 5.0

        self.dt = (
            1.0 / self.control_rate
        )

        #
        # Command timeout.
        #
        # If velocity commands stop arriving, force
        # the commanded velocity to zero.
        #
        self.command_timeout = 0.5

        self.last_command_time = None


        #
        # Latest commanded Cartesian velocity.
        #
        self.cartesian_velocity = np.zeros(3)

        #
        # Create the simulated robot.
        #
        self.robot = self.create_robot()

        #
        # Solve the initial robot configuration.
        #
        self.backbone = (
            solve_forward_kinematics_v2(
                self.robot,
            )
        )

        #
        # Subscribe to Cartesian velocity commands.
        #
        self.subscription = (
            self.create_subscription(
                Twist,
                "/ctr/cartesian_velocity",
                self.velocity_callback,
                1,
            )
        )

        #
        # Run the control loop at a fixed rate.
        #
        self.timer = self.create_timer(
            self.dt,
            self.control_callback,
        )

        self.get_logger().info(
            "CTR simulator started."
        )

        self.get_logger().info(
            f"Control rate: "
            f"{self.control_rate:.1f} Hz"
        )

    def create_robot(self):

        nitinol = Material(
            name="Nitinol",
            youngs_modulus=60e9,
            shear_modulus=23e9,
        )

        fiber = Material(
            name="SilicaFiber",
            youngs_modulus=15e9,
            shear_modulus=6.4e9,
        )

        outer = Tube(
            name="OuterTube",
            length=0.16,
            precurvature=15.0,
            outer_diameter=3.0e-3,
            inner_diameter=2.8e-3,
            material=nitinol,
        )

        middle = Tube(
            name="MiddleTube",
            length=0.18,
            precurvature=10.0,
            outer_diameter=2.0e-3,
            inner_diameter=1.8e-3,
            material=nitinol,
        )

        inner = Tube(
            name="InnerTube",
            length=0.20,
            precurvature=0.0,
            outer_diameter=1.6e-3,
            inner_diameter=0.0,
            material=fiber,
        )

        state = CTRState(
            insertions=[
                0.10,
                0.18,
                0.19,
            ],
            rotations=[
                np.pi / 2,
                0.0,
                0.0,
            ],
        )

        return ConcentricTubeRobot(
            tubes=[
                outer,
                middle,
                inner,
            ],
            state=state,
        )

    def velocity_callback(
        self,
        msg: Twist,
    ):

        #
        # Store the newest commanded Cartesian velocity.
        #
        self.cartesian_velocity = np.array([
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
        ])

        self.last_command_time = (
            self.get_clock().now()
        )

    def control_callback(self):
        
        #
        # Current ROS time for the command watchdog
        #
        now = self.get_clock().now()

        #
        # Stop motion if Cartesian commands have
        # stopped arriving. Command watchdog. 
        #
        if self.last_command_time is None:

            self.cartesian_velocity[:] = 0.0

        else:

            command_age = (
                now - self.last_command_time
            ).nanoseconds * 1e-9

            if command_age > self.command_timeout:

                self.cartesian_velocity[:] = 0.0

        #
        # If zero velocity is commanded, there is
        # no mechanics solve to perform.
        #
        if np.allclose(
            self.cartesian_velocity,
            0.0,
        ):
            return

        #
        # Convert velocity into the Cartesian
        # displacement requested for one nominal
        # control interval.
        #
        dx = (
            self.cartesian_velocity
            * self.dt
        )

        #
        # Use the current torsion solution to
        # warm-start the next mechanics solve.
        #
        torsion_initial_guess = (
            self.backbone
            .torsion_solution
            .base_theta_dot
        )

        control_start = (
            self.get_clock().now()
        )

        #
        # Compute the resolved-rate joint step.
        #
        dq = resolved_rate_step_v2(
            self.robot,
            dx,
            torsion_initial_guess=(
                torsion_initial_guess
            ),
        )

        #
        # Enforce insertion constraints.
        #
        dq = constrain_joint_step(
            self.robot,
            dq,
        )

        n = len(
            self.robot.tubes
        )

        #
        # Update simulated joint state.
        #
        self.robot.state.insertions += (
            dq[:n]
        )

        self.robot.state.rotations += (
            dq[n:]
        )

        #
        # Verify that the updated configuration
        # remains valid.
        #
        self.robot.validate_configuration()

        #
        # Recompute the robot shape using the
        # previous torsion solution as a warm start.
        #
        self.backbone = (
            solve_forward_kinematics_v2(
                self.robot,
                torsion_initial_guess=(
                    torsion_initial_guess
                ),
            )
        )

        control_end = (
            self.get_clock().now()
        )

        control_compute_time = (
            control_end - control_start
        ).nanoseconds * 1e-9

        #
        # Sample the backbone so we can inspect
        # the resulting tip position.
        #
        samples = sample_backbone(
            self.backbone,
            ds=1e-3,
        )

        tip = samples.position[-1]

        self.get_logger().info(
            "Tip: "
            f"[{tip[0]:+.5f}, "
            f"{tip[1]:+.5f}, "
            f"{tip[2]:+.5f}], "
            f"control={control_compute_time:.3f}s"
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = CTRSimulator()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()