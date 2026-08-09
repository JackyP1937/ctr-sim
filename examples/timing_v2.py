import time
from copy import deepcopy

import numpy as np

from ctr_sim import (
    Material,
    Tube,
    CTRState,
    ConcentricTubeRobot,
)

from ctr_sim.kinematics.intervals import (
    backbone_segments,
)

from ctr_sim.mechanics.torsion_v2 import (
    solve_torsion_shooting,
    solve_torsion_shooting_continuation,
    solve_torsion_v2,
    evaluate_segment_torsion_v2,
)

from ctr_sim.mechanics.curvature import (
    segment_curvature_v2,
)

from ctr_sim.mechanics.integration import (
    integrate_segment,
)

from ctr_sim.mechanics.forward_v2 import (
    solve_forward_kinematics_v2,
)

from ctr_sim.mechanics.sampling import (
    sample_backbone,
)

from ctr_sim.control.jacobian import (
    numerical_position_jacobian_v2,
)


# ----------------------------------------
# Materials
# ----------------------------------------

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


# ----------------------------------------
# Robot
# ----------------------------------------

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

robot = ConcentricTubeRobot(
    tubes=[
        outer,
        middle,
        inner,
    ],
    state=state,
)


# ----------------------------------------
# Settings
# ----------------------------------------

delta_insertion = 1e-4
delta_rotation = 1e-4


def elapsed(start):
    return time.perf_counter() - start


# ========================================
# 1. Segment-aware forward breakdown
# ========================================

start = time.perf_counter()

segments = backbone_segments(
    robot,
)

segment_time = elapsed(start)


start = time.perf_counter()

torsion_solution = solve_torsion_v2(
    robot,
)

torsion_time = elapsed(start)


position = np.zeros(3)
rotation = np.eye(3)

torsion_evaluation_time = 0.0
curvature_time = 0.0
integration_time = 0.0

for segment in segments:

    start = time.perf_counter()

    theta = evaluate_segment_torsion_v2(
        torsion_solution,
        segment,
    )

    torsion_evaluation_time += elapsed(
        start
    )


    start = time.perf_counter()

    curvature = segment_curvature_v2(
        theta,
        segment,
    )

    curvature_time += elapsed(
        start
    )


    start = time.perf_counter()

    _, position, rotation = integrate_segment(
        position,
        rotation,
        curvature,
        segment,
    )

    integration_time += elapsed(
        start
    )


manual_forward_time = (
    segment_time
    + torsion_time
    + torsion_evaluation_time
    + curvature_time
    + integration_time
)


# ========================================
# 2. Direct shooting timing
# ========================================

start = time.perf_counter()

direct_result = solve_torsion_shooting(
    robot,
)

direct_shooting_time = elapsed(
    start
)


# ========================================
# 3. Continuation timing
# ========================================

start = time.perf_counter()

continuation_result = (
    solve_torsion_shooting_continuation(
        robot,
        n_steps=10,
    )
)

continuation_time = elapsed(
    start
)


# ========================================
# 4. Complete forward solve
# ========================================

start = time.perf_counter()

backbone = solve_forward_kinematics_v2(
    robot,
)

forward_time = elapsed(start)


# ========================================
# 5. Backbone sampling
# ========================================

start = time.perf_counter()

samples = sample_backbone(
    backbone,
    ds=1e-3,
)

sampling_time = elapsed(start)


# ========================================
# 6. Numerical Jacobian
# ========================================

start = time.perf_counter()

J = numerical_position_jacobian_v2(
    robot,
)

jacobian_time = elapsed(start)


# ========================================
# 7. Individual Jacobian forward solves
# ========================================

n = len(robot.tubes)

perturbation_results = []


# Nominal
start = time.perf_counter()

nominal_backbone = (
    solve_forward_kinematics_v2(
        robot,
    )
)

nominal_time = elapsed(start)

perturbation_results.append(
    (
        "nominal",
        nominal_time,
    )
)


# Insertion perturbations
for i in range(n):

    beta_i = robot.state.insertions[i]
    tube_length = robot.tubes[i].length

    if beta_i + delta_insertion <= tube_length:

        robot_perturbed = deepcopy(
            robot
        )

        robot_perturbed.state.insertions[i] += (
            delta_insertion
        )

        difference_type = "forward"

    else:

        robot_perturbed = deepcopy(
            robot
        )

        robot_perturbed.state.insertions[i] -= (
            delta_insertion
        )

        difference_type = "backward"

    start = time.perf_counter()

    solve_forward_kinematics_v2(
        robot_perturbed,
    )

    solve_time = elapsed(start)

    perturbation_results.append(
        (
            f"beta_{i + 1} ({difference_type})",
            solve_time,
        )
    )


# Rotation perturbations
for i in range(n):

    robot_plus = deepcopy(
        robot
    )

    robot_plus.state.rotations[i] += (
        delta_rotation
    )

    start = time.perf_counter()

    solve_forward_kinematics_v2(
        robot_plus,
    )

    solve_time = elapsed(start)

    perturbation_results.append(
        (
            f"alpha_{i + 1}",
            solve_time,
        )
    )


manual_jacobian_time = sum(
    result[1]
    for result in perturbation_results
)


# ========================================
# Results
# ========================================

print()
print("=" * 70)
print("Mechanics V2 Timing")
print("=" * 70)


print()
print("Forward mechanics breakdown:")

print(
    f"  Backbone segmentation:       "
    f"{segment_time:10.6f} s"
)

print(
    f"  Segment-aware torsion:       "
    f"{torsion_time:10.6f} s"
)

print(
    f"  Segment torsion evaluation:  "
    f"{torsion_evaluation_time:10.6f} s"
)

print(
    f"  Segment curvature:           "
    f"{curvature_time:10.6f} s"
)

print(
    f"  Segment integration:         "
    f"{integration_time:10.6f} s"
)

print()
print(
    f"  Manual breakdown total:      "
    f"{manual_forward_time:10.6f} s"
)


print()
print("Torsion solver behavior:")

print(
    f"  Direct shooting:             "
    f"{direct_shooting_time:10.6f} s"
)

print(
    f"    success: "
    f"{direct_result.success}"
)

print(
    f"  10-step continuation:        "
    f"{continuation_time:10.6f} s"
)

print(
    f"    success: "
    f"{continuation_result.success}"
)


print()
print("Complete operations:")

print(
    f"  solve_forward_kinematics_v2: "
    f"{forward_time:10.6f} s"
)

print(
    f"  sample_backbone:             "
    f"{sampling_time:10.6f} s"
)

print(
    f"  numerical Jacobian V2:       "
    f"{jacobian_time:10.6f} s"
)


print()
print("=" * 70)
print("Jacobian Forward-Solve Breakdown")
print("=" * 70)

for name, solve_time in perturbation_results:

    print(
        f"  {name:<28}"
        f"{solve_time:10.6f} s"
    )

print()
print(
    f"  total:                       "
    f"{manual_jacobian_time:10.6f} s"
)


print()
print("=" * 70)
print("Numerical Results")
print("=" * 70)

print()
print("Jacobian:")

print(
    np.round(
        J,
        4,
    )
)

print()
print("Tip:")

print(
    np.round(
        samples.position[-1],
        6,
    )
)

print()























# import time
# from copy import deepcopy

# import numpy as np

# from ctr_sim import (
#     Material,
#     Tube,
#     CTRState,
#     ConcentricTubeRobot,
# )

# from ctr_sim.kinematics.intervals import (
#     backbone_segments,
# )

# from ctr_sim.mechanics.torsion import (
#     solve_torsion_bvp,
#     evaluate_segment_torsion,
# )

# from ctr_sim.mechanics.curvature import (
#     segment_curvature,
# )

# from ctr_sim.mechanics.integration import (
#     integrate_segment,
# )

# from ctr_sim.mechanics.forward_v2 import (
#     solve_forward_kinematics_v2,
# )

# from ctr_sim.mechanics.sampling import (
#     sample_backbone,
# )

# from ctr_sim.control.jacobian import (
#     numerical_position_jacobian_v2,
# )


# # ----------------------------------------
# # Materials
# # ----------------------------------------

# nitinol = Material(
#     name="Nitinol",
#     youngs_modulus=60e9,
#     shear_modulus=23e9,
# )

# fiber = Material(
#     name="SilicaFiber",
#     youngs_modulus=15e9,
#     shear_modulus=6.4e9,
# )


# # ----------------------------------------
# # Robot
# # ----------------------------------------

# outer = Tube(
#     name="OuterTube",
#     length=0.16,
#     precurvature=15.0,
#     outer_diameter=3.0e-3,
#     inner_diameter=2.8e-3,
#     material=nitinol,
# )

# middle = Tube(
#     name="MiddleTube",
#     length=0.18,
#     precurvature=10.0,
#     outer_diameter=2.0e-3,
#     inner_diameter=1.8e-3,
#     material=nitinol,
# )

# inner = Tube(
#     name="InnerTube",
#     length=0.20,
#     precurvature=0.0,
#     outer_diameter=1.6e-3,
#     inner_diameter=0.0,
#     material=fiber,
# )

# state = CTRState(
#     insertions=[0.10, 0.18, 0.19],
#     rotations=[np.pi / 2, 0.0, 0.0],
# )

# robot = ConcentricTubeRobot(
#     tubes=[outer, middle, inner],
#     state=state,
# )


# # ----------------------------------------
# # Settings
# # ----------------------------------------

# delta_insertion = 1e-4
# delta_rotation = 1e-4


# # ----------------------------------------
# # Timing helper
# # ----------------------------------------

# def elapsed(start):
#     return time.perf_counter() - start

# def print_mesh_diagnostics(
#     name,
#     solution,
#     robot,
#     window=1e-3,
# ):
#     """
#     Print where solve_bvp placed its adaptive mesh nodes.
#     """

#     print()
#     print(f"{name} mesh diagnostics")
#     print("-" * 50)

#     print(
#         "Total nodes:",
#         len(solution.x),
#     )

#     for i, beta in enumerate(
#         robot.state.insertions
#     ):

#         distances = np.abs(
#             solution.x - beta
#         )

#         count = np.sum(
#             distances <= window
#         )

#         closest_idx = np.argmin(
#             distances
#         )

#         closest_node = solution.x[
#             closest_idx
#         ]

#         print(
#             f"beta_{i + 1} = {beta:.6f}: "
#             f"{count} nodes within +/- {window:.4f} m, "
#             f"closest node = {closest_node:.9f}"
#         )

#     print()
#     print("Local mesh near tube tips:")

#     for i, beta in enumerate(
#         robot.state.insertions
#     ):

#         mask = (
#             np.abs(solution.x - beta)
#             <= window
#         )

#         local_nodes = solution.x[
#             mask
#         ]

#         print()
#         print(
#             f"beta_{i + 1} "
#             f"({beta:.6f}):"
#         )

#         print(
#             np.round(
#                 local_nodes,
#                 9,
#             )
#         )

# # ----------------------------------------
# # 1. Forward mechanics breakdown
# # ----------------------------------------

# start = time.perf_counter()

# segments = backbone_segments(robot)

# segment_time = elapsed(start)


# start = time.perf_counter()

# bvp_solution = solve_torsion_bvp(robot)

# torsion_time = elapsed(start)


# position = np.zeros(3)
# rotation = np.eye(3)

# torsion_evaluation_time = 0.0
# curvature_time = 0.0
# integration_time = 0.0

# for segment in segments:

#     start = time.perf_counter()

#     theta = evaluate_segment_torsion(
#         robot,
#         bvp_solution,
#         segment,
#     )

#     torsion_evaluation_time += elapsed(start)

#     start = time.perf_counter()

#     curvature = segment_curvature(
#         robot,
#         theta,
#         segment,
#     )

#     curvature_time += elapsed(start)

#     start = time.perf_counter()

#     _, position, rotation = integrate_segment(
#         position,
#         rotation,
#         curvature,
#         segment,
#     )

#     integration_time += elapsed(start)


# manual_forward_time = (
#     segment_time
#     + torsion_time
#     + torsion_evaluation_time
#     + curvature_time
#     + integration_time
# )


# # ----------------------------------------
# # 2. Complete forward solve
# # ----------------------------------------

# start = time.perf_counter()

# backbone = solve_forward_kinematics_v2(
#     robot,
# )

# forward_time = elapsed(start)


# # ----------------------------------------
# # 3. Backbone sampling
# # ----------------------------------------

# start = time.perf_counter()

# samples = sample_backbone(
#     backbone,
#     ds=1e-3,
# )

# sampling_time = elapsed(start)


# # ----------------------------------------
# # 4. Existing numerical Jacobian
# # ----------------------------------------

# start = time.perf_counter()

# J = numerical_position_jacobian_v2(
#     robot,
# )

# jacobian_time = elapsed(start)


# # ----------------------------------------
# # 5. Manual Jacobian timing breakdown
# # ----------------------------------------

# n = len(robot.tubes)

# perturbation_results = []


# # Nominal solve
# start = time.perf_counter()

# nominal_backbone = solve_forward_kinematics_v2(
#     robot,
# )

# nominal_samples = sample_backbone(
#     nominal_backbone,
#     ds=1e-4,
# )

# x0 = nominal_samples.position[-1]

# nominal_time = elapsed(start)


# # Insertion perturbations
# for i in range(n):

#     robot_plus = deepcopy(robot)

#     robot_plus.state.insertions[i] += delta_insertion

#     start = time.perf_counter()

#     perturbed_backbone = solve_forward_kinematics_v2(
#         robot_plus,
#     )

#     perturbed_samples = sample_backbone(
#         perturbed_backbone,
#         ds=1e-4,
#     )

#     x_plus = perturbed_samples.position[-1]

#     total_time = elapsed(start)

#     perturbation_results.append(
#         (
#             f"beta_{i + 1}",
#             total_time,
#             x_plus,
#         )
#     )


# # Rotation perturbations
# for i in range(n):

#     robot_plus = deepcopy(robot)

#     robot_plus.state.rotations[i] += delta_rotation

#     start = time.perf_counter()

#     perturbed_backbone = solve_forward_kinematics_v2(
#         robot_plus,
#     )

#     perturbed_samples = sample_backbone(
#         perturbed_backbone,
#         ds=1e-4,
#     )

#     x_plus = perturbed_samples.position[-1]

#     total_time = elapsed(start)

#     perturbation_results.append(
#         (
#             f"alpha_{i + 1}",
#             total_time,
#             x_plus,
#         )
#     )


# manual_jacobian_time = (
#     nominal_time
#     + sum(
#         result[1]
#         for result in perturbation_results
#     )
# )


# # ----------------------------------------
# # 6. BVP-only timing for each perturbation
# # ----------------------------------------

# bvp_results = []


# # Nominal BVP
# start = time.perf_counter()

# nominal_bvp = solve_torsion_bvp(
#     robot,
# )

# nominal_bvp_time = elapsed(start)

# bvp_results.append(
#     (
#         "nominal",
#         nominal_bvp_time,
#         nominal_bvp.success,
#         len(nominal_bvp.x),
#         getattr(nominal_bvp, "niter", None),
#     )
# )


# # Insertion perturbations
# for i in range(n):

#     robot_plus = deepcopy(robot)

#     robot_plus.state.insertions[i] += delta_insertion

#     start = time.perf_counter()

#     solution = solve_torsion_bvp(
#         robot_plus,
#     )

#     solve_time = elapsed(start)

#     bvp_results.append(
#         (
#             f"beta_{i + 1}",
#             solve_time,
#             solution.success,
#             len(solution.x),
#             getattr(solution, "niter", None),
#         )
#     )


# # Rotation perturbations
# for i in range(n):

#     robot_plus = deepcopy(robot)

#     robot_plus.state.rotations[i] += delta_rotation

#     start = time.perf_counter()

#     solution = solve_torsion_bvp(
#         robot_plus,
#     )

#     solve_time = elapsed(start)

#     bvp_results.append(
#         (
#             f"alpha_{i + 1}",
#             solve_time,
#             solution.success,
#             len(solution.x),
#             getattr(solution, "niter", None),
#         )
#     )


# # ----------------------------------------
# # 7. Adaptive mesh diagnostics
# # ----------------------------------------

# print_mesh_diagnostics(
#     "Nominal",
#     nominal_bvp,
#     robot,
# )


# robot_beta3 = deepcopy(robot)

# robot_beta3.state.insertions[2] += (
#     delta_insertion
# )

# beta3_bvp = solve_torsion_bvp(
#     robot_beta3,
# )

# print_mesh_diagnostics(
#     "beta_3 perturbation",
#     beta3_bvp,
#     robot_beta3,
# )

# # ----------------------------------------
# # Results
# # ----------------------------------------

# print()
# print("=" * 70)
# print("Mechanics V2 Timing")
# print("=" * 70)

# print()
# print("Forward mechanics breakdown:")

# print(
#     f"  Backbone segmentation:       "
#     f"{segment_time:10.6f} s"
# )

# print(
#     f"  Torsion BVP:                 "
#     f"{torsion_time:10.6f} s"
# )

# print(
#     f"  Segment torsion evaluation:  "
#     f"{torsion_evaluation_time:10.6f} s"
# )

# print(
#     f"  Segment curvature:           "
#     f"{curvature_time:10.6f} s"
# )

# print(
#     f"  Segment integration:         "
#     f"{integration_time:10.6f} s"
# )

# print()
# print(
#     f"  Manual breakdown total:      "
#     f"{manual_forward_time:10.6f} s"
# )


# print()
# print("Complete operations:")

# print(
#     f"  solve_forward_kinematics_v2: "
#     f"{forward_time:10.6f} s"
# )

# print(
#     f"  sample_backbone:             "
#     f"{sampling_time:10.6f} s"
# )

# print(
#     f"  numerical Jacobian V2:       "
#     f"{jacobian_time:10.6f} s"
# )


# print()
# print("=" * 70)
# print("Manual Jacobian Forward-Solve Breakdown")
# print("=" * 70)

# print()
# print(
#     f"  nominal:                     "
#     f"{nominal_time:10.6f} s"
# )

# for name, solve_time, _ in perturbation_results:

#     print(
#         f"  {name:<28}"
#         f"{solve_time:10.6f} s"
#     )

# print()
# print(
#     f"  total:                       "
#     f"{manual_jacobian_time:10.6f} s"
# )


# print()
# print("=" * 70)
# print("BVP-Only Breakdown")
# print("=" * 70)

# print()
# print(
#     f"{'case':<12}"
#     f"{'time (s)':>12}"
#     f"{'nodes':>10}"
#     f"{'iters':>10}"
#     f"{'success':>10}"
# )

# print("-" * 54)

# for (
#     name,
#     solve_time,
#     success,
#     nodes,
#     iterations,
# ) in bvp_results:

#     print(
#         f"{name:<12}"
#         f"{solve_time:>12.6f}"
#         f"{nodes:>10}"
#         f"{str(iterations):>10}"
#         f"{str(success):>10}"
#     )


# print()
# print("=" * 70)
# print("Numerical Results")
# print("=" * 70)

# print()
# print("Jacobian:")
# print(
#     np.round(
#         J,
#         4,
#     )
# )

# print()
# print("Tip:")
# print(
#     np.round(
#         samples.position[-1],
#         6,
#     )
# )

# print()