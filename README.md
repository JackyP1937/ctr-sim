# ctr-sim

A Python library for modeling and simulating concentric tube robots (CTRs).

![CTR Backbone](docs/images/backbone.png)

`ctr-sim` provides tools for robot modeling, forward mechanics, and visualization using a Cosserat rod formulation. The project is being developed as an open-source robotics library with an emphasis on clean software architecture, numerical methods, and extensibility.

> **Current status:** Active development (v0.1)

---

## Features

- Concentric tube robot modeling
- Tube material and geometric properties
- Robot state representation (insertions and rotations)
- Tube interval and backbone segmentation utilities
- Unloaded forward mechanics
  - Tube torsion boundary value solver
  - Resultant curvature computation
  - Cosserat rod backbone integration
- 3D backbone visualization using Matplotlib
- Automated unit and integration tests using `pytest`

---

## Installation

Clone the repository:

```bash
git clone https://github.com/<your-username>/ctr-sim.git
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

```
ctr_sim/
│
├── mechanics/
│   ├── forward.py
│   ├── integration.py
│   └── torsion.py
│
├── kinematics/
│   ├── intervals.py
│   └── transforms.py
│
├── visualization/
│
└── ...
```

```
examples/
```

```
tests/
```

---

## Current Capabilities

- Robot modeling
- Forward kinematics
- Backbone reconstruction
- 3D visualization
- Automated testing
- Validated unloaded forward kinematics against an independent MATLAB implementation

---

## Roadmap

- MATLAB validation
- Improved plotting utilities
- Workspace analysis
- Animation
- MuJoCo visualization
- Inverse kinematics
- ROS integration

---

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.