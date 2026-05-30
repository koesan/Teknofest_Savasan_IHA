"""
Savaşan İHA - Reacquisition Module

Hedef kaybolduğunda yeniden tespit stratejileri.
"""

from __future__ import annotations

import time
from typing import Optional, Tuple

from core.logger import get_logger


class Reacquisition:
    """
    Hedef kaybedilince arama stratejileri.

    Stratejiler:
    1. Predicted ROI: Son hız vektörüne göre tahmini pozisyon
    2. Expanding ROI: Giderek genişleyen arama penceresi
    3. Full-frame: Tüm kareyi tara
    """

    def __init__(
        self,
        frame_width: int = 1280,
        frame_height: int = 720,
        initial_expand: float = 2.5,
        expand_rate: float = 1.3,
        max_expand: float = 8.0,
    ):
        self._log = get_logger()
        self._fw = frame_width
        self._fh = frame_height
        self._initial_expand = initial_expand
        self._expand_rate = expand_rate
        self._max_expand = max_expand

        # State
        self._lost_time: Optional[float] = None
        self._expand_factor = initial_expand
        self._last_known_center: Optional[Tuple[float, float]] = None
        self._last_known_velocity: Optional[Tuple[float, float]] = None
        self._last_known_size: Optional[Tuple[float, float]] = None
        self._attempt_count = 0

    def start(
        self,
        center: Tuple[float, float],
        velocity: Tuple[float, float],
        size: Tuple[float, float],
    ):
        """Reacquisition başlat."""
        self._lost_time = time.time()
        self._expand_factor = self._initial_expand
        self._last_known_center = center
        self._last_known_velocity = velocity
        self._last_known_size = size
        self._attempt_count = 0
        self._log.info("REACQ",
            f"START: last_pos=({center[0]:.0f},{center[1]:.0f}) "
            f"vel=({velocity[0]:.0f},{velocity[1]:.0f}) size=({size[0]:.0f}x{size[1]:.0f})")

    def get_search_roi(self) -> Optional[Tuple[int, int, int, int]]:
        """
        Bir sonraki arama ROI'si.
        Her çağrıda expand_factor büyür.
        None döndürürse full-frame kullanılmalı.
        """
        if self._last_known_center is None or self._lost_time is None:
            return None

        self._attempt_count += 1
        elapsed = time.time() - self._lost_time

        # Hız tahminiyle merkez güncelle
        cx, cy = self._last_known_center
        vx, vy = self._last_known_velocity or (0, 0)
        pred_cx = cx + vx * elapsed
        pred_cy = cy + vy * elapsed

        # Boyut * expand factor
        sw, sh = self._last_known_size or (64, 64)
        hw = sw * self._expand_factor / 2
        hh = sh * self._expand_factor / 2

        roi = (
            int(max(0, pred_cx - hw)),
            int(max(0, pred_cy - hh)),
            int(min(self._fw, pred_cx + hw)),
            int(min(self._fh, pred_cy + hh)),
        )

        # Expand for next attempt
        self._expand_factor = min(self._expand_factor * self._expand_rate, self._max_expand)

        # ROI artık full-frame'e eşit veya büyükse, None döndür
        if (roi[2] - roi[0]) >= self._fw * 0.9 and (roi[3] - roi[1]) >= self._fh * 0.9:
            self._log.info("REACQ", f"ROI FULL-FRAME: attempt={self._attempt_count} elapsed={elapsed:.1f}s")
            return None

        self._log.debug("REACQ",
            f"ROI: attempt={self._attempt_count} pred=({pred_cx:.0f},{pred_cy:.0f}) "
            f"expand={self._expand_factor:.1f}x roi=({roi[0]:.0f},{roi[1]:.0f},{roi[2]:.0f},{roi[3]:.0f})",
            debounce=0.3)
        return roi

    def reset(self):
        """Reacquisition durumunu sıfırla."""
        self._lost_time = None
        self._expand_factor = self._initial_expand
        self._last_known_center = None
        self._last_known_velocity = None
        self._last_known_size = None
        self._attempt_count = 0
