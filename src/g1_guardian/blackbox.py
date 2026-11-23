"""Blackbox: the last few seconds of every control tick, dumped on events.

Recording is a deque append per tick. Dumps happen in a background thread
from a copy, so a latch never costs the control loop a file write.
"""

from __future__ import annotations

import json
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple


class Blackbox:
    def __init__(self, directory: Optional[Path], *, seconds: float, rate_hz: float, keep_files: int = 50) -> None:
        self.directory = None if directory is None else Path(directory)
        self.ring: Deque[Tuple[Any, ...]] = deque(maxlen=max(1, int(seconds * rate_hz)))
        self.keep_files = keep_files
        self.fields: List[str] = []
        self._threads: List[threading.Thread] = []

    def record(self, row: Tuple[Any, ...]) -> None:
        self.ring.append(row)

    def dump(self, reason: str, header: Dict[str, Any]) -> Optional[threading.Thread]:
        if self.directory is None:
            return None
        rows = list(self.ring)
        safe = "".join(c if c.isalnum() or c in "-_" else "_" for c in reason)[:60]
        path = self.directory / f"blackbox_{time.strftime('%Y%m%d_%H%M%S')}_{int(time.time() * 1000) % 1000:03d}_{safe}.jsonl"

        def write() -> None:
            try:
                self.directory.mkdir(parents=True, exist_ok=True)
                with path.open("w", encoding="utf-8") as f:
                    f.write(json.dumps(dict(header, reason=reason, fields=self.fields, rows=len(rows))) + "\n")
                    for row in rows:
                        f.write(json.dumps(row) + "\n")
                files = sorted(self.directory.glob("blackbox_*.jsonl"))
                for old in files[: max(0, len(files) - self.keep_files)]:
                    old.unlink(missing_ok=True)
            except OSError as exc:
                print(f"[g1_guardian][blackbox][WARN] {exc}", flush=True)

        t = threading.Thread(target=write, name="blackbox_dump", daemon=True)
        t.start()
        self._threads = [x for x in self._threads if x.is_alive()] + [t]
        return t

    def wait(self, timeout: float = 5.0) -> None:
        for t in list(self._threads):
            t.join(timeout)
