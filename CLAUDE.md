# robot-dev-platform

Python platform for the Unitree G1 / G1D. It covers the web UI, calibration (hand-eye, TCP, payload), perception, tasks, and the arm control loop.

- `src/robot_platform/`: the platform (one web process, `serve.py`).
- `src/g1_guardian/`: the body-side execution guardian. It is the only program that writes `rt/lowcmd` (G1D) or `rt/arm_sdk` (G1). It uses only the standard library and must never import `robot_platform`.

Design background: `docs/architecture-review.md` (Chinese, 2026-11-23; §十二 has the owner's answers, §十三 the revised conclusions). Current work items are listed in HANDOFF.md in the handoff package.

## Safety invariants: check every change against these

1. **G1D drops its arm when `rt/lowcmd` stops.** Never add a code path that stops sending frames while engaged. The only exception is an explicit release that ramps stiffness to 0 first (`LowCmdReleaseRamp`, `GuardianCore` state `releasing`). E-STOP, cancel, errors, a stale setpoint and link loss all mean *decelerate and hold*.
2. **One writer per robot.** With `--backend guardian`, this process must not create a lowcmd or arm_sdk publisher (enforced in `tests/test_guardian_backend.py`). Never add a second program that writes to the actuators.
3. **Any hold restores full stiffness.** `kp_scale` can only lower stiffness (≤ 1), applies only while its setpoint is valid, and changes are ramped. `kp_scale = 0` (teach) makes the commanded position follow the measured one.
4. **Guardian config ↔ platform profile.** `src/g1_guardian/configs/<robot>.json` must match `robots/<robot>/robot.json` in gains (waist motors use `waist.lock_kp/lock_kd`), arm motor groups, waist lock motors, `tau_ff_limit` and `idle_motors`. `test_guardian_config_matches_the_platform_profile` checks this. Change both files together.
5. **Wire protocol.** Any change to the guardian UDP JSON format must bump `g1_guardian.PROTOCOL_VERSION`. The refactor repo's `hal/guardian_arm_driver.py` speaks this protocol too.
6. Collision checking is **not** a safety function (unstable, off for customers). The safety functions are the position, velocity and acceleration limits.

## Guardian code rules

- `core.py` has no I/O and takes time as an argument. Every new behaviour needs a fake-clock test in `tests/test_guardian_core.py`. If it involves the loop or the network, it also needs a real-UDP test in `tests/test_guardian_service.py`.
- The service loop is single-threaded. Never block it, and never let an exception end it.
- `dds.py` has only been run against a fake SDK. Treat it as unverified until it has run on hardware.

## Commands

```bash
python -m pytest tests                       # whole suite
python -m pytest tests/test_guardian_*.py    # guardian + platform guardian backend
python serve.py --backend mock --port 10184  # UI without hardware
python -m g1_guardian.client --host 192.168.123.164 watch   # guardian status
```

Tests that can fail for reasons unrelated to your change:

- `tests/test_rgbd.py::test_rgbd_handeye_solve_recovers_camera_pose` fails when the installed cv2 has no `calibrateHandEye`. It passes with `opencv-python-headless==4.11.0.86`.
- `tests/test_ik_robust.py::test_goto_with_locked_work_pose_uses_robust_ik` requires the mock control loop to stay at or above 80 Hz during a robust-IK goto, so it fails when the machine is busy (on `main` too). Re-run it on an idle machine before treating it as a regression.

## Conventions

- Platform code comments, UI strings and README are in Chinese. Guardian code and docs are in English, except the guardian README.
- Default backends stay `unitree_lowcmd` (G1D) and `unitree_arm_sdk` (G1) until the guardian has passed the hardware checklist in `src/g1_guardian/README.md`.
- Nothing may be tested on hardware unless the arm is mechanically supported and someone can cut power.
