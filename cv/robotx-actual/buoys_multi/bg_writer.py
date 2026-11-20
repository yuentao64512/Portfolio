"""bg_writer: the observer's slow disk writes off the inference loop (v0.1.86.7, WP43 Tier 1.2; docs/PROMPT_COMPUTE_NIGHT_2026-11-11.md §1.2).

One daemon thread, one bounded queue (maxsize 4). The loop only enqueues; the thread runs the jobs in order. On a full queue the OLDEST
PREVIEW is dropped (the live feed skips a picture); a job marked keep (the RGB + depth pairs) is never dropped - when nothing can be dropped
the put waits up to keep_wait_s and then runs the job inline (late, never lost). latest_status.json is NOT here: it stays in the loop
(small, and cv_publish / the guard read it in order). Stdlib only (the observer may run in the students' read-only venv).
"""
from __future__ import annotations

import collections
import threading
import time
from typing import Callable, Optional


class BgWriter:
    def __init__(self, maxsize: int = 4, keep_wait_s: float = 0.5):
        self.maxsize, self.keep_wait_s = int(maxsize), float(keep_wait_s)
        self.q: "collections.deque" = collections.deque()
        self.cv = threading.Condition()
        self.stats = {"done": 0, "dropped_previews": 0, "inline": 0, "errors": 0, "max_job_ms": 0.0, "last_error": None}
        self._stop = False
        self.t = threading.Thread(target=self._run, daemon=True, name="observer-writer")
        self.t.start()

    def put(self, kind: str, fn: Callable[[], None], keep: bool = False) -> bool:
        """Enqueue fn (kind 'preview' | 'pair' | ...); False when it was run inline (the queue full of jobs that cannot be dropped)."""
        deadline = time.monotonic() + self.keep_wait_s
        with self.cv:
            while len(self.q) >= self.maxsize:
                prev = next((j for j in self.q if not j[2]), None)
                if prev is not None:
                    self.q.remove(prev)
                    self.stats["dropped_previews"] += 1
                    continue
                if not keep:
                    self.stats["dropped_previews"] += 1          # a preview behind a queue of pairs: skip this picture
                    return True
                left = deadline - time.monotonic()
                if left <= 0:
                    break
                self.cv.wait(left)
            if len(self.q) < self.maxsize:
                self.q.append((kind, fn, keep))
                self.cv.notify_all()
                return True
        self.stats["inline"] += 1
        self._do(fn)
        return False

    def _do(self, fn) -> None:
        t0 = time.perf_counter()
        try:
            fn()
            self.stats["done"] += 1
        except Exception as e:                                      # a failed write never kills the observer
            self.stats["errors"] += 1
            self.stats["last_error"] = f"{e.__class__.__name__}: {e}"
        self.stats["max_job_ms"] = max(self.stats["max_job_ms"], round((time.perf_counter() - t0) * 1000.0, 1))

    def _run(self) -> None:
        while True:
            with self.cv:
                while not self.q and not self._stop:
                    self.cv.wait(0.5)
                if self._stop and not self.q:
                    return
                kind, fn, keep = self.q.popleft()
                self.cv.notify_all()
            self._do(fn)

    def close(self, timeout_s: float = 5.0) -> None:
        """Finish what is queued (the pairs) and stop."""
        with self.cv:
            self._stop = True
            self.cv.notify_all()
        self.t.join(timeout_s)

    def public(self) -> dict:
        with self.cv:
            return dict(self.stats, queued=len(self.q))
