from ctr_sim.backbone import Backbone
from ctr_sim.backbone_samples import BackboneSamples


def sample_backbone(
    backbone: Backbone,
    ds: float,
) -> BackboneSamples:
    """
    Sample a segment-wise backbone representation.
    """
    raise NotImplementedError