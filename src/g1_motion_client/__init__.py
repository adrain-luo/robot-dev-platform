"""g1_motion_client: talk to the motion core process over its local sockets (phase 2).

Standard library and ``g1_contracts`` only, so the refactor's system Python
and the platform's conda environment can both use it.

    from g1_motion_client import CoreConnection, MotionClient
    client = MotionClient(CoreConnection("/run/g1-motion", client="skill:demo"))
    with client.lease([Resource.RIGHT_ARM], "skill:demo/task-1") as lease:
        lease.goal(Resource.RIGHT_ARM, [q])

The wire format is in ``g1_contracts.wire``.
"""

from g1_motion_client.connection import CoreConnection, CoreError, CoreUnavailable
from g1_motion_client.client import MotionClient, RemoteLease

__all__ = ["CoreConnection", "CoreError", "CoreUnavailable", "MotionClient", "RemoteLease"]
