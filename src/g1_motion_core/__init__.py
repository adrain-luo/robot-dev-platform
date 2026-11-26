"""g1_motion_core: the motion core as its own process (phase 2a).

- ``server``: the socket server (standard library and ``g1_contracts`` only).
  It serves any object with the in-process ``MotionCore`` methods, so it is
  tested against the real core with the mock backend.
- ``service``: the process entry point. It assembles the platform's ArmLoop,
  MotionCore, backend, collision gate and payload, registers the platform
  calls the web needs, and runs the server::

      python -m g1_motion_core.service --backend mock --socket-dir /tmp/g1-motion

Task and web code must not import this package (tests/test_import_rules.py);
they talk to it through ``g1_motion_client``.
"""
