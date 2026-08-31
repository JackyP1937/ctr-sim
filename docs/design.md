# ctr-sim Software Design

## Vision

`ctr-sim` is a Python library and ROS 2 simulation framework for modeling, simulating, controlling, and teleoperating concentric tube robots (CTRs).

The project is designed around a reusable mechanics library that remains independent of visualization, ROS 2, teleoperation, and any particular deployment strategy. ROS 2 provides an interface around the core mechanics and control algorithms rather than being embedded within them.

## Design Philosophy

1. Keep the robot model independent of external interfaces.
2. Compute derived quantities instead of storing redundant state.
3. Use SI units throughout the mechanics and control code.
4. Build small, focused classes with clear responsibilities.
5. Separate data structures from algorithms.
6. Keep mechanics independent of actuation strategy.
7. Keep visualization and ROS 2 communication outside the core mechanics solver.
8. Preserve continuous mechanics solutions and sample them only when needed.

## Software Architecture

```text
Xbox Controller
      |
      | XInput
      v
Windows joy_sender.py
      |
      | UDP
      v
ROS 2 joy_receiver
      |
      | /joy
      v
cartesian_teleop
      |
      | /ctr/cartesian_velocity
      v
ctr_simulator
      |
      +-----------------------------+
      |                             |
      v                             v
CTR Core Library              ROS 2 State Output
      |                             |
      +-- robot model               +-- /ctr/backbone
      +-- mechanics                 +-- /ctr/tip_pose
      +-- Jacobian                  +-- /ctr/tip_trajectory
      +-- resolved-rate control     +-- /tf
      +-- constraints               +-- /tf_static
      |                             |
      +-------------+---------------+
                    |
                    v
                   RViz
```

The CTR core library is the center of the project. It can be used independently of ROS 2.

The ROS 2 layer handles controller input, control-loop scheduling, state publication, TF broadcasting, and RViz visualization.

## Core Data Structures

### Material

Represents the physical material of a structural element.

Stores properties including:

- name;
- Young's modulus;
- shear modulus.

Example materials include Nitinol and silica optical fiber.

### Tube

Represents one physical concentric tube or solid cylindrical element.

Stores:

- name;
- length;
- precurvature;
- inner diameter;
- outer diameter;
- material.

Derived mechanical quantities are computed from the tube geometry and material properties.

A `Tube` contains physical properties but no current configuration state.

### CTRState

Represents the current robot configuration.

Stores:

- tube insertions `beta`;
- tube rotations `alpha`.

For an `n`-tube robot, the controller uses the joint ordering

```text
q = [beta_1 ... beta_n alpha_1 ... alpha_n]
```

Geometry and material properties remain in the corresponding `Tube` objects.

### ConcentricTubeRobot

Represents a complete CTR.

Stores:

- an ordered list of tubes;
- the current `CTRState`.

The robot is responsible for configuration and geometry validation.

### Segment

Represents one contiguous backbone interval over which the active tube set is constant.

Tube proximal and distal boundaries partition the robot into these segments.

### SegmentSolution

Represents the solved mechanics state for one backbone segment.

Stores:

- the corresponding `Segment`;
- tube angular state;
- resultant curvature;
- the dense numerical IVP solution for backbone position and orientation.

The dense IVP solution allows the backbone state to be evaluated continuously within the segment without fixing the mechanics solution to a particular visualization sampling resolution.

### Backbone

Represents the complete segment-aware forward-mechanics solution.

The backbone contains the ordered `SegmentSolution` objects describing the robot from base to tip together with the solved torsional state used by the mechanics pipeline.

The backbone can subsequently be sampled at any desired spatial resolution for visualization or analysis.

## Tube Ordering

Tubes are stored from outermost to innermost.

```text
Index 0       outermost tube
...
Index N - 1   innermost tube
```

Robot-state ordering follows the same convention.

## Tube Insertion Convention

For tube `i`, insertion `beta_i` defines the distal tube-tip location relative to the robot base.

```text
beta_i = 0
    tube tip is located at the robot base

beta_i > 0
    tube tip extends beyond the robot base
```

The valid range is

```text
0 <= beta_i <= L_i
```

where `L_i` is the tube length.

A tube therefore occupies the arc-length interval

```text
[beta_i - L_i, beta_i]
```

relative to the robot base.

## Backbone Segmentation

Tube boundaries determine where the set of active tubes changes.

The mechanics pipeline constructs contiguous backbone segments such that the active tube set remains constant within each segment.

Conceptually:

```text
tube boundaries
      |
      v
sorted geometric intervals
      |
      v
Segment 0
Segment 1
Segment 2
...
      |
      v
segment-aware mechanics
```

This representation avoids treating changing tube geometry as if it were uniform along the entire backbone.

## Kinematics

The `kinematics` package contains geometric utilities that do not solve the CTR mechanics problem.

Examples include:

- rigid-body transformations;
- interval operations;
- backbone segmentation utilities.

Mechanics and control algorithms build on these geometric operations.

## Mechanics

The `mechanics` package implements the unloaded CTR mechanics pipeline.

The segment-aware V2 forward solver performs approximately:

```text
Robot geometry + configuration
            |
            v
Backbone segmentation
            |
            v
Segment-aware torsion solution
            |
            v
Segment torsion evaluation
            |
            v
Resultant segment curvature
            |
            v
Spatial backbone integration
            |
            v
SegmentSolution objects
            |
            v
Backbone
```

### Torsion

Tube torsion is solved as a boundary-value problem using a shooting formulation.

Continuation is used when necessary to improve convergence from difficult initial guesses.

During interactive operation, the previous successful torsion solution is reused as an initial guess for nearby configurations. This warm start substantially reduces the cost of repeated forward-mechanics solves.

### Curvature

Within each mechanics segment, the active tube set and solved tube angular state determine the resultant backbone curvature.

Because the active tube set is constant within a segment, curvature is evaluated consistently with the local robot geometry.

### Backbone Integration

The spatial backbone state consists of position and orientation.

Each segment is integrated as an initial-value problem, producing a dense continuous solution. The terminal state of one segment initializes the next segment so that the complete backbone remains continuous.

### Sampling

The mechanics solution is independent of visualization resolution.

`sample_backbone()` evaluates the continuous segment solutions at a requested spatial spacing and returns sampled position and orientation arrays.

When only the tip position is needed, the final segment's dense IVP solution is evaluated directly at its endpoint rather than sampling the complete backbone.

## Solver Philosophy

The mechanics solver computes the robot equilibrium shape for a specified configuration `(alpha, beta)`.

It does not assume a particular actuation strategy such as:

- follow-the-leader deployment;
- staged insertion;
- joystick teleoperation;
- autonomous planning.

Actuation and control are intentionally separated from mechanics.

This allows the same forward solver to support interactive control, numerical Jacobians, validation examples, and future planning algorithms.

## Cartesian Control

The control layer implements numerical task-space control on top of the mechanics solver.

### Numerical Position Jacobian

The Cartesian position Jacobian is approximated with finite differences:

```text
J(q) = d p_tip / d q
```

with columns ordered as

```text
[beta_1 ... beta_n alpha_1 ... alpha_n]
```

Insertion perturbations use forward differences when possible and backward differences near the maximum insertion limit. Rotation derivatives use forward differences.

Each perturbation requires another nonlinear forward-mechanics solve, making Jacobian computation the primary computational bottleneck.

### Resolved-Rate Control

For a desired Cartesian displacement `dx`, the controller computes

```text
dq = J(q)^+ dx
```

using the numerical pseudoinverse.

The resulting joint increment is then passed through insertion constraints before the robot state is updated.

### Jacobian Caching

The ROS simulator caches the numerical Jacobian and reuses it for several nearby active control steps.

This reduces the number of nonlinear mechanics solves required during teleoperation.

The scheduling policy belongs to the ROS simulator rather than the core Jacobian routine: the core control code can either compute a new Jacobian or accept a previously computed one.

## ROS 2 Layer

The ROS 2 package is located under:

```text
ros2_ws/src/ctr_teleop/
```

Its major nodes are:

### `joy_receiver`

Receives Xbox controller state over UDP from the Windows-side sender and publishes ROS 2 `Joy` messages.

### `cartesian_teleop`

Converts joystick input into Cartesian tip-velocity commands.

The right trigger acts as a deadman control.

Cartesian commands are expressed in the `ctr_base` frame.

### `ctr_simulator`

Owns the simulated robot state and:

- executes the resolved-rate controller;
- calls the core CTR mechanics library;
- manages Jacobian caching;
- enforces the command watchdog;
- publishes backbone visualization;
- publishes the tip pose;
- publishes the tip trajectory;
- broadcasts static and dynamic TF transforms.

Control/mechanics computation and ROS state publication use separate timers so the latest available state can continue to be published independently of the nonlinear mechanics computation rate.

## Coordinate Frames

The current TF hierarchy is:

```text
world
  |
  v
ctr_base
  |
  v
ctr_tip
```

`world -> ctr_base` is currently a static identity transform.

`ctr_base -> ctr_tip` is dynamically computed from the current forward-mechanics solution.

Cartesian teleoperation commands are expressed along the X, Y, and Z axes of `ctr_base`, not the moving `ctr_tip` frame.

## Visualization

The project supports both Python/Matplotlib visualization and ROS 2/RViz visualization.

The RViz interface displays:

- the current backbone;
- individual mechanics segments using distinct colors;
- `world`, `ctr_base`, and `ctr_tip` coordinate frames;
- the accumulated Cartesian tip trajectory.

Backbone visualization uses a `MarkerArray` because each mechanics segment is represented independently.

The tip trajectory uses a separate `LINE_STRIP` marker and maintains a bounded history.

## Performance Strategy

Interactive CTR mechanics are computationally demanding because a numerical Jacobian requires multiple nonlinear forward solves.

The current implementation improves interactive performance through:

1. warm-started torsion solves;
2. direct tip extraction from dense IVP solutions;
3. Jacobian reuse across nearby control configurations;
4. avoiding mechanics solves when the commanded Cartesian velocity is zero;
5. independent ROS state-publication timing.

These optimizations preserve the nonlinear forward-mechanics model while reducing unnecessary repeated computation.

## Testing and Validation

The project includes automated tests for:

- materials and tube geometry;
- robot state and validation;
- geometric intervals and segmentation;
- torsion mechanics;
- segment curvature;
- forward mechanics;
- backbone sampling;
- numerical Jacobians;
- resolved-rate control;
- insertion constraints.

The forward mechanics have also been compared against independent MATLAB validation cases.

Timing and Jacobian-reuse examples are retained in `examples/` to document performance behavior and optimization decisions.

## Current Limitations

The current implementation intentionally remains a research/development simulator.

Notable limitations include:

- unloaded mechanics only;
- no external contact or distributed environmental loading;
- computationally expensive finite-difference Jacobians;
- approximate Jacobian reuse between refreshes;
- shooting/continuation convergence is not guaranteed for arbitrary robot geometries;
- insertion constraints are applied after the unconstrained resolved-rate calculation;
- no hardware CTR interface;
- the Windows Xbox sender remains a separate process from the ROS 2 launch system.

These boundaries are kept explicit so future extensions can be added without coupling them unnecessarily to the core mechanics solver.