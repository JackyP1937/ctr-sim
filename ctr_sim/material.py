from dataclasses import dataclass


@dataclass(frozen=True)
class Material:
    """Mechanical properties of a material."""

    name: str
    youngs_modulus: float
    shear_modulus: float














# """
# material.py

# Defines materials used in concentric tube robots
# """

# from dataclasses import dataclass

# @dataclass(frozen=True)
# class Material:
#     """
#     Physical material properties
    
#     Parameters
#     ----------
#     name: str
#         human-readable material name

#     youngs_modulus: float
#         Young's modulus (Pa)

#     shear_modulus: float
#         Shear modulus (Pa)
#     """

#     name: str

#     youngs_modulus: float

#     shear_modulus: float

#     def __str__(self) -> str:
#         return (
#             f"{self.name}\n"
#             f" Young's Modulus : {self.youngs_modulus:.3e} Pa\n"
#             f" Shear Modulus : {self.shear_modulus:.3e} Pa"
#         )