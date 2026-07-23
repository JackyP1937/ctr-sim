# ctr-sim

A Python library for modeling, simulating, and controlling concentric tube robots (CTRs).

![CTR Backbone](docs/images/backbone.png)

`ctr-sim` is an open-source robotics library for concentric tube robots built with an emphasis on clean software architecture, numerical methods, and extensibility. The project provides tools for robot modeling, mechanics, visualization, and the foundations of task-space control.

> **Current status:** Active development (v0.1)

---

## Features

### Robot Modeling

- Material definitions
- Tube geometry and mechanical properties
- Robot configuration and state representation
- Tube interval and backbone segmentation utilities

### Mechanics

- Unloaded forward kinematics
- Tube torsion boundary value solver
- Resultant backbone curvature computation
- Cosserat rod backbone integration
- Backbone reconstruction

### Control

- Numerical position Jacobian
- Initial resolved-rate inverse kinematics framework

### Visualization

- 3D backbone visualization using Matplotlib

### Software Quality

- Automated unit and integration tests using `pytest`
- Forward mechanics validated against an independent MATLAB implementation

---

## Installation

Clone the repository:

```bash
git clone https://github.com/<your-github-username>/ctr-sim.git
cd ctr-sim
```

Create and activate a virtual environment:

```bash
python -m venv .venv
source .venv/bin/activate
```

Install the package:

```bash
pip install -e .
```

---

## Quick Start

```python
import numpy as np

from ctr_sim import (
    Material,
    Tube,
    CTRState,
    ConcentricTubeRobot,
)

from ctr_sim.mechanics import solve_forward_kinematics
from ctr_sim.visualization import plot_backbone


nitinol = Material(
    name="Nitinol",
    youngs_modulus=60e9,
    shear_modulus=23e9,
)

outer = Tube(
    name="Outer",
    length=0.16,
    precurvature=15.0,
    outer_diameter=3.0e-3,
    inner_diameter=2.8e-3,
    material=nitinol,
)

middle = Tube(
    name="Middle",
    length=0.18,
    precurvature=10.0,
    outer_diameter=2.0e-3,
    inner_diameter=1.8e-3,
    material=nitinol,
)

inner = Tube(
    name="Inner",
    length=0.20,
    precurvature=0.0,
    outer_diameter=1.6e-3,
    inner_diameter=0.0,
    material=nitinol,
)

state = CTRState(
    insertions=[0.10, 0.12, 0.14],
    rotations=[0.0, np.pi / 6, 0.0],
)

robot = ConcentricTubeRobot(
    tubes=[outer, middle, inner],
    state=state,
)

solution = solve_forward_kinematics(robot)

plot_backbone(solution)
```

---

## Project Structure

```text
ctr_sim/
│
├── mechanics/
│   ├── forward.py
│   ├── integration.py
│   └── torsion.py
│
├── control/
│   ├── jacobian.py
│   └── resolved_rate.py
│
├── kinematics/
│   ├── intervals.py
│   └── transforms.py
│
├── visualization/
│
└── ...
```

```text
examples/
```

```text
tests/
```

---

## Current Capabilities

- Robot modeling
- Unloaded forward kinematics
- Backbone reconstruction
- 3D visualization
- Numerical position Jacobian
- Initial resolved-rate inverse kinematics
- Automated unit and integration testing
- MATLAB validation of unloaded forward mechanics

---

## Current Development

The project is currently focused on refactoring the forward mechanics to use a segment-aware backbone representation. This will provide a more physically faithful treatment of changing tube boundaries and improve numerical Jacobians, inverse kinematics, and task-space teleoperation.

---

## Roadmap

See [ROADMAP.md](ROADMAP.md) for the current development roadmap.

---

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.