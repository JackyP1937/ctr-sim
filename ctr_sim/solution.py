from dataclasses import dataclass
import numpy as np
from ctr_sim.backbone import Backbone


@dataclass
class CTRSolution:
    """
    Complete solution of the CTR forward mechanics problem.
    """

    # Backbone quantities (at all points along backbone)
    backbone: Backbone

    # Tube torsion angles θ(s)
    torsion: np.ndarray

    # Tip quantities
    tip_position: np.ndarray
    tip_rotation: np.ndarray