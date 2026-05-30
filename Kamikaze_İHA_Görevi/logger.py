from __future__ import annotations

import os
import sys
import threading
import time
from typing import Optional


class Logger:
    def __init__(self, log_path: str, level: str = "INFO", console: bool = True):
        self._lock = threading.Lock()
        self._console = console
        self._level = level.upper()
        self._start = time.time()
        self._last: dict[str, float] = {}
        os.makedirs(os.path.dirname(log_path) or ".", exist_ok=True)
        self._file = open(log_path, "w", encoding="utf-8", buffering=1)

    def info(self, category: str, message: str, debounce: float = 0.0):
        self._log("INFO", category, message, debounce)

    def warn(self, category: str, message: str, debounce: float = 0.0):
        self._log("WARN", category, message, debounce)

    def error(self, category: str, message: str, debounce: float = 0.0):
        self._log("ERROR", category, message, debounce)

    def debug(self, category: str, message: str, debounce: float = 0.0):
        if self._level == "DEBUG":
            self._log("DEBUG", category, message, debounce)

    def close(self):
        self._file.close()

    def _log(self, level: str, category: str, message: str, debounce: float):
        now = time.time()
        if debounce > 0:
            key = f"{category}:{message[:48]}"
            last = self._last.get(key, 0.0)
            if now - last < debounce:
                return
            self._last[key] = now

        stamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now))
        ms = int((now % 1) * 1000)
        elapsed = now - self._start
        line = f"[{stamp}.{ms:03d}] [{elapsed:8.2f}s] [{level}] [{category}] {message}"
        with self._lock:
            self._file.write(line + "\n")
            if self._console:
                print(line, file=sys.stderr)


_LOGGER: Optional[Logger] = None


def init_logger(log_path: str, level: str = "INFO", console: bool = True) -> Logger:
    global _LOGGER
    _LOGGER = Logger(log_path=log_path, level=level, console=console)
    return _LOGGER


def get_logger() -> Logger:
    global _LOGGER
    if _LOGGER is None:
        _LOGGER = Logger(log_path="mission_log.txt")
    return _LOGGER
