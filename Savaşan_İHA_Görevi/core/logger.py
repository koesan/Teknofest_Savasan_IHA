"""
Savaşan İHA - Structured Logger
Kategorize edilmiş, zaman damgalı, CSV-uyumlu loglama sistemi.
"""

from __future__ import annotations

import os
import sys
import time
import threading
from typing import Optional


class Logger:
    """Kategori filtrelemeli, iş parçacığı güvenli (thread-safe) yapılandırılmış günlükleyici."""

    CATEGORIES = {"DETECT", "TRACK", "CONTROL", "SERVO", "DIST", "LOCK", "FSM", "VEHICLE",
                  "MISSION", "CAMERA", "CONFIG", "SYSTEM", "LOOP", "REACQ"}

    def __init__(
        self,
        log_path: str = "system_log.txt",
        level: str = "INFO",
        console: bool = True,
    ):
        self._lock = threading.Lock()
        self._console = console
        self._start_time = time.time()
        self._level = level.upper()
        self._log_path = log_path
        self._file = None
        self._last_msgs: dict[str, float] = {}  # yinelenen mesajları engellemek için (debounce)

        if log_path:
            os.makedirs(os.path.dirname(log_path) or ".", exist_ok=True)
            self._file = open(log_path, "w", encoding="utf-8", buffering=1)

    # ---- genel metotlar ----------------------------------------------------

    def info(self, category: str, message: str, debounce: float = 0.0):
        self._log("INFO", category, message, debounce)

    def warn(self, category: str, message: str, debounce: float = 0.0):
        self._log("WARN", category, message, debounce)

    def error(self, category: str, message: str, debounce: float = 0.0):
        self._log("ERROR", category, message, debounce)

    def debug(self, category: str, message: str, debounce: float = 0.0):
        if self._level == "DEBUG":
            self._log("DEBUG", category, message, debounce)

    def elapsed(self) -> float:
        return time.time() - self._start_time

    def close(self):
        if self._file:
            self._file.close()
            self._file = None

    # ---- özel metotlar ----------------------------------------------------

    def _log(self, level: str, category: str, message: str, debounce: float):
        now = time.time()
        if debounce > 0:
            key = f"{category}:{message[:40]}"
            last = self._last_msgs.get(key, 0.0)
            if now - last < debounce:
                return
            self._last_msgs[key] = now

        ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now))
        ms = int((now % 1) * 1000)
        elapsed = now - self._start_time
        line = f"[{ts}.{ms:03d}] [{elapsed:8.2f}s] [{level}] [{category}] {message}"

        with self._lock:
            if self._file:
                self._file.write(line + "\n")
            if self._console:
                print(line, file=sys.stderr)

    def __del__(self):
        self.close()


# Modül seviyesinde tekil nesne (singleton)
_logger: Optional[Logger] = None


def init_logger(log_path: str = "system_log.txt", level: str = "INFO", console: bool = True) -> Logger:
    global _logger
    _logger = Logger(log_path=log_path, level=level, console=console)
    return _logger


def get_logger() -> Logger:
    global _logger
    if _logger is None:
        _logger = Logger()
    return _logger
