"""
Savasan IHA - Competition Server Communications Stub

Gercek yarisma API'sine gecmeden once ic arayuzu sartname formatina
yaklastirmak icin kullanilir.
"""

from __future__ import annotations

import time
from typing import Optional

import numpy as np

from core.logger import get_logger


class ServerComms:
    """Yarisma sunucusu haberlesme arabirimi."""

    def __init__(
        self,
        min_telemetry_hz: float = 1.0,
        max_telemetry_hz: float = 2.0,
    ):
        self._log = get_logger()
        self._min_period = 1.0 / max(max_telemetry_hz, 0.1)
        self._max_period = 1.0 / max(min_telemetry_hz, 0.1)
        self._last_telemetry_time = 0.0
        self._last_lock_time = 0.0
        self._server_time: float = time.time()

    @property
    def server_time(self) -> float:
        return self._server_time

    def send_telemetry(
        self,
        lat: float,
        lon: float,
        alt: float,
        heading: float,
        mode: str,
        is_autonomous: int = 1,
        is_locking: int = 0,
        target_bbox: Optional[np.ndarray] = None,
        now: Optional[float] = None,
    ):
        """Sartname mantigina yakin telemetri paketi olustur."""
        now = now or time.time()
        if now - self._last_telemetry_time < self._min_period:
            return

        target_cx = target_cy = target_w = target_h = 0
        if target_bbox is not None:
            x1, y1, x2, y2 = [float(v) for v in target_bbox]
            target_cx = int(round((x1 + x2) / 2.0))
            target_cy = int(round((y1 + y2) / 2.0))
            target_w = int(round(max(x2 - x1, 0.0)))
            target_h = int(round(max(y2 - y1, 0.0)))

        packet = {
            "takim_numarasi": 0,
            "iha_enlem": lat,
            "iha_boylam": lon,
            "iha_irtifa": alt,
            "iha_dikilme": 0.0,
            "iha_yonelme": heading,
            "iha_yatis": 0.0,
            "iha_hiz": 0.0,
            "iha_batarya": 100,
            "iha_otonom": int(bool(is_autonomous)),
            "iha_kilitlenme": int(bool(is_locking)),
            "hedef_merkez_X": target_cx,
            "hedef_merkez_Y": target_cy,
            "hedef_genislik": target_w,
            "hedef_yukseklik": target_h,
            "gps_saati": self._clock_dict(now),
        }
        self._last_telemetry_time = now
        self._log.debug(
            "SYSTEM",
            f"Telemetri: lat={lat:.7f} lon={lon:.7f} alt={alt:.1f} "
            f"yon={heading:.1f} otonom={packet['iha_otonom']} "
            f"kilit={packet['iha_kilitlenme']} hedef=({target_cx},{target_cy},{target_w}x{target_h})",
            debounce=0.6,
        )
        return packet

    def send_lock_packet(
        self,
        target_id: str,
        lock_rect: tuple,
        lock_duration: float,
        now: Optional[float] = None,
    ):
        """Kilitlenme bitisinden sonra tek seferlik lock paketi."""
        now = now or time.time()
        packet = {
            "kilitlenmeBitisZamani": self._clock_dict(now),
            "otonom_kilitlenme": 1,
            "hedef_id": target_id,
            "lock_rect": {
                "x1": int(lock_rect[0]),
                "y1": int(lock_rect[1]),
                "x2": int(lock_rect[2]),
                "y2": int(lock_rect[3]),
            },
            "sure_sn": round(float(lock_duration), 3),
        }
        self._log.info(
            "SYSTEM",
            f"KILITLENME PAKETI: hedef={target_id} sure={lock_duration:.2f}s rect={lock_rect}",
        )
        self._last_lock_time = now
        return packet

    def update_server_time(self, server_time: float):
        """Sunucu saatini disaridan guncelle."""
        self._server_time = server_time

    @staticmethod
    def _clock_dict(ts: float) -> dict:
        local = time.localtime(ts)
        return {
            "saat": local.tm_hour,
            "dakika": local.tm_min,
            "saniye": local.tm_sec,
            "milisaniye": int((ts % 1) * 1000),
        }
