# ctr-sim Software Design

## Vision

`ctr-sim` is a Python library for modeling, simulating, and controlling concentric tube robots (CTRs).

The project is designed around a reusable mechanics solver that is independent of visualization, ROS 2, teleoperation, or any particular deployment strategy.

---

# Design Philosophy

1. Keep the robot model independent of interfaces.
2. Compute derived quantities instead of storing them.
3. Use SI units everywhere.
4. Build small, focused classes with one responsibility.
5. Separate data structures from algorithms.
6. Mechanics are independent of actuation strategy.

---

# Software Architecture

                Xbox Controller
                       │
                       ▼
                Windows Bridge
                       │
                       ▼
                    ROS2 Node
                       │
                       ▼
               CTR Core Library
                       │
      ┌────────────────┴────────────────┐
      ▼                                 ▼
 Mechanics Solver                Visualization
      │                                 │
      ▼                                 ▼
 MuJoCo Simulation               Jupyter / Plots

The CTR Core Library is the center of the project.

---

# Core Classes

Material

Represents the physical material of a structural element.

Examples

- Nitinol
- Stainless Steel
- Silica Optical Fiber

---

Tube

Represents one physical concentric tube (or solid cylindrical element).

Stores

- name
- length
- precurvature
- inner diameter
- outer diameter
- material

Computes

- wall thickness
- I
- J
- EI
- GJ

Tube contains no configuration information.

---

CTRState

Represents the current robot configuration.

Stores

- β (tube insertions)
- α (tube rotations)

Configuration only.

No geometry.

---

ConcentricTubeRobot

Represents a complete robot.

Stores

- ordered list of tubes
- current robot state

Robot validation includes

- tube nesting
- tube length ordering
- state dimensions

---

Backbone

Represents the solved backbone geometry.

Stores

- arc length
- backbone position
- backbone orientation

---

Segment

Represents one contiguous backbone interval.

Within a segment the active tube set is constant.

---

CTRSolution

Complete solution of the forward mechanics problem.

Stores

- Backbone
- torsion θ(s)
- tip position
- tip orientation

---

# Tube Ordering

The robot stores tubes from outermost to innermost.

Index 0
    Outermost tube

Index N−1
    Innermost tube

Robot state ordering follows the same convention.

---

# Tube Insertion Convention

Insertion βᵢ is defined as the distance from the robot base to the distal tip
of tube i.

βᵢ = 0

    Tube tip is located at the robot base.

βᵢ > 0

    Tube tip has advanced beyond the robot base.

Valid range

0 ≤ βᵢ ≤ Lᵢ

---

# Geometry

Each tube occupies the interval

    [βᵢ − Lᵢ , βᵢ]

The backbone is partitioned into contiguous segments.

Each segment contains a constant set of active tubes.

---

# Kinematics

The kinematics package contains geometric algorithms.

Examples

- rigid body transformations
- backbone segmentation

These algorithms do not solve mechanics.

---

# Mechanics

The mechanics package implements the unloaded Cosserat rod formulation.

Primary components

- torsion BVP
- curvature computation
- backbone integration
- forward solver

The mechanics solver accepts

- tube geometry
- material properties
- current robot state

and computes

- backbone shape
- tube torsion
- tip pose

---

# Solver Philosophy

The mechanics solver computes the robot shape for a given robot
configuration (α, β).

The solver makes no assumptions about

- follow-the-leader deployment
- staged insertion
- teleoperation
- autonomous planning

Actuation strategy is intentionally separated from mechanics.

---

# Project Roadmap

## Phase 1
- [x] Project setup
- [x] Git repository
- [x] ROS2 controller bridge
- [x] Python package

## Phase 2
- [x] Material class
- [x] Tube class
- [x] CTRState
- [x] ConcentricTubeRobot
- [x] Backbone
- [x] Segment
- [x] CTRSolution

## Phase 3
- [ ] Torsion BVP
- [ ] Backbone integration
- [ ] Forward mechanics solver

## Phase 4
- [ ] MuJoCo visualization
- [ ] ROS2 teleoperation

## Phase 5
- [ ] Inverse kinematics
- [ ] Motion planning
- [ ] Hardware interface