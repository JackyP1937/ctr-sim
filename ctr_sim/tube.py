from dataclasses import dataclass
from ctr_sim.material import Material
from math import pi

@dataclass
class Tube:
    """
    Represents a single concentric tube or solid cylindrical element.

    A Tube stores the fixed geometric properties of a physical component,
    including its dimensions, precurvature, and material.

    Derived mechanical properties such as the area moment of inertia (I),
    polar moment of inertia (J), bending stiffness (EI), and torsional
    stiffness (GJ) are computed automatically from the stored geometry
    and material properties.

    This class represents the physical tube only. It does not store the
    tube's current insertion or rotation within a robot.
    """

    name: str
    
    length: float

    precurvature: float
    
    outer_diameter: float
    
    inner_diameter: float

    material: Material

    def __post_init__(self):
        """Validate tube geometry."""

        if self.inner_diameter >= self.outer_diameter:
            raise ValueError(
                "Inner diameter must be smaller than outer diameter."
            )

        elif self.inner_diameter < 0:
            raise ValueError(
                "Inner diameter must be greater than or equal to zero."
            )

        elif self.outer_diameter <= 0:
            raise ValueError(
                "Outer diameter must be positive."
            )

        elif self.length <= 0:
            raise ValueError(
                "Tube length must be positive."
            )

        elif self.precurvature < 0:
            raise ValueError(
                "Precurvature cannot be negative."
            )

    @property
    def I(self) -> float:
        """Area moment of inertia (m^4)."""
        return (
            (pi / 64) * (self.outer_diameter**4 - self.inner_diameter**4)
        )

    @property
    def J(self) -> float:
        """Polar moment of inertia (m^4)."""
        return (
            (pi / 32) * (self.outer_diameter**4 - self.inner_diameter**4)
        )    

    @property
    def EI(self) -> float:
        """bending stiffness EI (N·m²)"""
        return self.material.youngs_modulus * self.I

    @property
    def GJ(self) -> float:
        """torsional rigidity GJ (N·m²)"""
        return self.material.shear_modulus * self.J

    @property
    def wall_thickness(self) -> float:
        """Tube wall thickness (m)"""
        return (self.outer_diameter - self.inner_diameter) / 2

    




    
