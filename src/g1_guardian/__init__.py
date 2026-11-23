"""g1_guardian: the body-side execution guardian for Unitree G1 / G1D.

One small process on the robot's own computer is the only writer of
rt/lowcmd (G1D) or rt/arm_sdk (G1). Box-side programs send it joint
setpoints over UDP; it runs a fixed-rate loop that

- enforces position, velocity and acceleration limits and gain caps,
- holds the last commanded position whenever setpoints stop, expire,
  the link to the Box drops, lowstate goes stale, or E-STOP is pressed,
- never stops sending frames by itself: only an explicit, ramped release
  does (on G1D the arm drops as soon as frames stop).

Only the standard library is needed for the core, the client and the
tests; the Unitree SDK is imported lazily by the DDS adapters.
"""

__version__ = "0.1.0"
PROTOCOL_VERSION = 1
