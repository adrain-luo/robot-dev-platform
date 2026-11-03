#!/usr/bin/env python3
"""Read-only lowstate smoke test. Does not publish any commands.

Usage:
  cd project/g1d/arm_control
  python3 read_lowstate.py
  python3 read_lowstate.py --arm left --hz 2
  python3 read_lowstate.py --once --timeout 8
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from config_loader import load_settings
import instance as instances
from dds.lowstate import LowStateReader
from model.layout import format_q_table, get_arm


def main() -> int:
    p = argparse.ArgumentParser(description="G1D read-only rt/lowstate viewer")
    p.add_argument("--instance", default=None, help="机器实例名（instances/<名字>），默认按 ARM_INSTANCE / instances/default")
    p.add_argument("--arm", choices=("right", "left", "both"), default="right")
    p.add_argument("--iface", default=None, help="override network interface")
    p.add_argument("--timeout", type=float, default=None, help="first-sample timeout (s)")
    p.add_argument("--hz", type=float, default=5.0, help="print rate when streaming")
    p.add_argument("--once", action="store_true", help="print one sample and exit")
    p.add_argument("--rad", action="store_true", help="print radians instead of degrees")
    args = p.parse_args()

    settings = load_settings(instances.resolve(args.instance))
    iface = args.iface or settings.network_interface
    timeout = args.timeout if args.timeout is not None else settings.lowstate_timeout_s
    arms = ("right", "left") if args.arm == "both" else (args.arm,)

    print(
        f"[read_lowstate] iface={iface} topic={settings.state_topic} "
        f"arms={','.join(arms)} (no publisher)"
    )
    reader = LowStateReader(
        network_interface=iface,
        state_topic=settings.state_topic,
        dds_domain=settings.dds_domain,
        timeout_s=timeout,
        sdk_path=settings.unitree_sdk_path,
    )
    print(f"[read_lowstate] first sample ok, age={reader.age_s():.3f}s")

    period = 1.0 / max(args.hz, 0.1)
    try:
        while True:
            for name in arms:
                layout = get_arm(name)
                st = reader.read_arm(layout)
                print(
                    f"--- {name}  mode_machine={st.mode_machine}  "
                    f"age={reader.age_s():.3f}s ---"
                )
                for line in format_q_table(layout, st.q, degrees=not args.rad):
                    print(line)
            if args.once:
                return 0
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n[read_lowstate] stopped")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
