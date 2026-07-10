from dataclasses import dataclass
import numpy as np


@dataclass
class Backbone:
    """
    Backbone geometry of the CTR.
    """

    s: np.ndarray

    position: np.ndarray

    rotation: np.ndarray