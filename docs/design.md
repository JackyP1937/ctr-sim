# ctr-sim Software Design

## Vision

`ctr-sim` is a Python-based software library for modeling, simulating, and controlling concentric tube robots (CTRs).

The project is organized into independent layers so that the core robotics algorithms can be reused without ROS 2 or MuJoCo.

---

# Design Philosophy

The project follows five guiding principles.

1. Keep the robot model independent of interfaces.
2. Compute derived quantities instead of storing them.
3. Use SI units everywhere.
4. Build small, focused classes with one responsibility.
5. Separate physics, simulation, and visualization.

---

# Architecture

                  Xbox Controller
                         │
                         ▼
                Windows Controller Bridge
                         │
                         ▼
                     ROS2 Interface
                         │
                         ▼
              CTR Core Python Library
                         │
          ┌──────────────┴──────────────┐
          ▼                             ▼
      MuJoCo                     Jupyter Notebook
          │
          ▼
    Visualization

The CTR Core Library is the center of the project.
Everything else depends on it.

---

# Core Classes

Material
    Represents the physical material of a component.

Tube
    Represents one precurved concentric tube.

ConcentricTubeRobot
    Represents a collection of concentric tubes.

Future classes may include

    Kinematics
    Mechanics
    Trajectory
    Controller
    Simulator

---

# Material

Responsibilities

- Store material properties
- Provide mechanical constants

Input Properties

- name
- Young's modulus (E)
- Shear modulus (G)

Future Properties

- density
- Poisson's ratio
- color
- thermal expansion coefficient

Material knows nothing about robots.

---

# Tube

Responsibilities

Represents one physical tube.

Input Properties

- name
- length
- precurvature
- inner diameter
- outer diameter
- material

Derived Properties

- area moment of inertia (I)
- polar moment of inertia (J)
- bending stiffness (EI)
- torsional stiffness (GJ)
- wall thickness

The user never enters derived quantities.

---

# ConcentricTubeRobot

Responsibilities

Represents a complete robot.

Stores

- ordered list of tubes

Future responsibilities

- joint variables
- tube rotations
- tube translations
- forward kinematics
- robot state

Robot knows nothing about ROS2 or MuJoCo.

---

# Units

All quantities use SI units.

Length                 meters

Diameter               meters

Curvature              1/m

Young's modulus        Pa

Shear modulus          Pa

Moment of inertia      m⁴

EI                     N·m²

GJ                     N·m²

---

# Software Layers

Layer 1

CTR Core Library

Contains only robotics mathematics.

No ROS.
No MuJoCo.

Layer 2

Simulation

Uses the CTR library to update MuJoCo.

Layer 3

Interfaces

ROS2

Xbox controller

Future GUI

---

# Future Goals

- Forward kinematics
- Mechanics solver
- MuJoCo simulation
- Teleoperation
- Inverse kinematics
- Motion planning
- Hardware interface