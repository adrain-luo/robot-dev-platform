"""g1_contracts: typed interfaces between G1 / G1D components.

Standard library only, and no imports of any other project package, so every
repository can depend on it. ``motion`` is the interface between skills /
tasks and the motion core (design review section 7). The body-side guardian's
UDP protocol is separate (``g1_guardian.protocol``) and is not defined here.

Any incompatible change to these types must bump ``CONTRACT_VERSION``.
"""

from g1_contracts.motion import CONTRACT_VERSION

__all__ = ["CONTRACT_VERSION"]
