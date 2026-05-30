"""
Savasan IHA - Lock Evaluator

TEKNOFEST 2026 otonom kilitlenme kosullarini frame bazli degerlendirir.

Temel kurallar:
  - Hedef boyutu yatay veya dikeyde en az %5
  - Hedef merkezi ve kilitlenme dortgeni merkezi hedef vurus alaninda
  - Kilitlenme dortgeni hedefin en az %90'ini kapsar
  - Merkez sapmasi hedef boyutunun yarisindan buyuk olamaz
  - Kilitlenme 4 saniye boyunca korunmalidir
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from core.logger import get_logger


@dataclass
class LockStatus:
    """HUD ve gorev mantigi icin kilitlenme durumu."""

    is_locked: bool = False
    just_locked: bool = False
    lock_duration: float = 0.0
    lock_progress: float = 0.0
    valid_streak_duration: float = 0.0

    in_target_area: bool = False
    in_lock_zone: bool = False
    target_size_ok: bool = False
    coverage_ok: bool = False
    center_offset_ok: bool = False
    all_criteria_met: bool = False

    target_bbox: Optional[Tuple[int, int, int, int]] = None
    lock_rect: Optional[Tuple[int, int, int, int]] = None
    target_center_px: Tuple[float, float] = (0.0, 0.0)
    lock_center_px: Tuple[float, float] = (0.0, 0.0)
    center_offset_px: Tuple[float, float] = (0.0, 0.0)

    target_width_ratio: float = 0.0
    target_height_ratio: float = 0.0
    target_size_ratio: float = 0.0
    coverage_ratio: float = 0.0


class LockEvaluator:
    """
    Sartname uyumlu kilitlenme degerlendirmesi.

    AK: Kamera Gorus Alani (tum frame)
    AV: Hedef Vurus Alani
    AH: Kilitlenme Dortgeni
    HH: Hedef Hava Araci
    """

    def __init__(
        self,
        frame_width: int = 1280,
        frame_height: int = 720,
        roi_width: int = 640,
        roi_height: int = 384,
        target_area_margin_x: float = 0.25,
        target_area_margin_y: float = 0.10,
        lock_rect_padding: float = 0.08,
        min_target_size_ratio: float = 0.05,
        min_coverage_ratio: float = 0.90,
        max_center_offset_ratio: float = 0.5,
        required_lock_duration: float = 4.0,
        lock_window_duration: float = 5.0,
        hold_grace_time: float = 0.20,
        bbox_scale_factor: float = 0.0,
    ):
        self._log = get_logger()
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.roi_width = roi_width
        self.roi_height = roi_height

        mx = int(round(target_area_margin_x * frame_width))
        my = int(round(target_area_margin_y * frame_height))
        self.target_area = (mx, my, frame_width - mx, frame_height - my)

        self._padding = max(lock_rect_padding, 0.0)
        self._min_size_ratio = min_target_size_ratio
        self._min_coverage = min_coverage_ratio
        self._max_center_offset = max_center_offset_ratio
        self._required_duration = required_lock_duration
        self._window_duration = lock_window_duration
        self._grace_time = hold_grace_time
        self._bbox_scale_factor = max(bbox_scale_factor, 0.0)

        self._lock_start: Optional[float] = None
        self._last_criteria_met_time: Optional[float] = None
        self._is_locked = False
        self._lock_duration = 0.0
        self._total_locks = 0

    @property
    def total_locks(self) -> int:
        return self._total_locks

    def reset(self):
        """Kilitleme durumunu temizle."""
        self._lock_start = None
        self._last_criteria_met_time = None
        self._is_locked = False
        self._lock_duration = 0.0
        self._log.info("LOCK", "Lock evaluator reset")

    def evaluate(
        self,
        target_bbox: Optional[np.ndarray],
        now: Optional[float] = None,
    ) -> LockStatus:
        """Verilen hedef kutusunu sartnameye gore degerlendir."""
        now = now or time.time()
        status = LockStatus()

        if target_bbox is None:
            self._update_timing(criteria_met=False, now=now)
            status.lock_duration = self._lock_duration
            status.valid_streak_duration = self._lock_duration
            status.lock_progress = min(self._lock_duration / self._required_duration, 1.0)
            status.is_locked = self._is_locked
            return status

        x1, y1, x2, y2 = self._clip_bbox(target_bbox)
        if x2 <= x1 or y2 <= y1:
            self._update_timing(criteria_met=False, now=now)
            status.lock_duration = self._lock_duration
            status.valid_streak_duration = self._lock_duration
            status.lock_progress = min(self._lock_duration / self._required_duration, 1.0)
            status.is_locked = self._is_locked
            return status

        tw = float(x2 - x1)
        th = float(y2 - y1)
        tcx = float((x1 + x2) / 2.0)
        tcy = float((y1 + y2) / 2.0)

        status.target_bbox = (int(x1), int(y1), int(x2), int(y2))
        status.target_center_px = (tcx, tcy)

        # Değerlendirme için kullanılacak yapay boyutlar (örneğin %25 daha büyük)
        scale_mult = 1.0 + self._bbox_scale_factor
        eval_tw = tw * scale_mult
        eval_th = th * scale_mult

        status.target_width_ratio = eval_tw / float(self.frame_width)
        status.target_height_ratio = eval_th / float(self.frame_height)
        
        status.target_size_ratio = max(status.target_width_ratio, status.target_height_ratio)
        status.target_size_ok = status.target_size_ratio >= self._min_size_ratio

        ax1, ay1, ax2, ay2 = self.target_area
        status.in_target_area = (ax1 <= tcx <= ax2) and (ay1 <= tcy <= ay2)

        lock_x1, lock_y1, lock_x2, lock_y2 = self._build_lock_rect(x1, y1, x2, y2)
        status.lock_rect = (lock_x1, lock_y1, lock_x2, lock_y2)
        lock_cx = float((lock_x1 + lock_x2) / 2.0)
        lock_cy = float((lock_y1 + lock_y2) / 2.0)
        status.lock_center_px = (lock_cx, lock_cy)

        coverage = self._compute_coverage(
            np.array([x1, y1, x2, y2], dtype=float),
            float(lock_x1), float(lock_y1), float(lock_x2), float(lock_y2),
        )
        status.coverage_ratio = coverage
        status.coverage_ok = coverage >= self._min_coverage

        dx = abs(lock_cx - tcx)
        dy = abs(lock_cy - tcy)
        status.center_offset_px = (dx, dy)
        status.center_offset_ok = (
            dx <= eval_tw * self._max_center_offset
            and dy <= eval_th * self._max_center_offset
        )

        lock_in_area = (ax1 <= lock_cx <= ax2) and (ay1 <= lock_cy <= ay2)
        status.in_lock_zone = lock_in_area and status.in_target_area

        status.all_criteria_met = (
            status.target_size_ok
            and status.in_target_area
            and status.in_lock_zone
            and status.coverage_ok
            and status.center_offset_ok
        )

        if not status.all_criteria_met:
            failed = []
            if not status.target_size_ok:
                failed.append(
                    f"boyut={status.target_size_ratio*100:.1f}%<{self._min_size_ratio*100:.1f}%"
                )
            if not status.in_target_area:
                failed.append("hedef_vurus_alani_disinda")
            if not status.in_lock_zone:
                failed.append("kilit_dortgen_merkezi_disinda")
            if not status.coverage_ok:
                failed.append(f"kapsama={coverage*100:.1f}%<{self._min_coverage*100:.1f}%")
            if not status.center_offset_ok:
                failed.append(
                    f"merkez_sapmasi=({dx:.1f},{dy:.1f})>"
                    f"izin=({eval_tw*self._max_center_offset:.1f},{eval_th*self._max_center_offset:.1f})"
                )
            self._log.debug(
                "LOCK",
                f"KRITER BASARISIZ: {', '.join(failed)} | "
                f"bbox=({tw:.0f}x{th:.0f}) merkez=({tcx:.0f},{tcy:.0f}) eval_bbox=({eval_tw:.0f}x{eval_th:.0f})",
                debounce=0.4,
            )

        self._update_timing(criteria_met=status.all_criteria_met, now=now)

        status.just_locked = False
        if self._lock_duration >= self._required_duration and not self._is_locked:
            self._is_locked = True
            self._total_locks += 1
            status.just_locked = True
            self._log.info(
                "LOCK",
                f">>> KILITLENME BASARILI duration={self._lock_duration:.2f}s "
                f"total_locks={self._total_locks} <<<",
            )

        status.is_locked = self._is_locked
        status.lock_duration = self._lock_duration
        status.valid_streak_duration = self._lock_duration
        status.lock_progress = min(self._lock_duration / self._required_duration, 1.0)

        return status

    def _clip_bbox(self, target_bbox: np.ndarray) -> Tuple[int, int, int, int]:
        x1, y1, x2, y2 = [float(v) for v in target_bbox]
        x1 = int(np.clip(round(x1), 0, self.frame_width - 1))
        y1 = int(np.clip(round(y1), 0, self.frame_height - 1))
        x2 = int(np.clip(round(x2), x1 + 1, self.frame_width))
        y2 = int(np.clip(round(y2), y1 + 1, self.frame_height))
        return x1, y1, x2, y2

    def _build_lock_rect(self, x1: int, y1: int, x2: int, y2: int) -> Tuple[int, int, int, int]:
        tw = float(x2 - x1)
        th = float(y2 - y1)
        pad_w = tw * self._padding
        pad_h = th * self._padding

        lock_x1 = int(np.clip(np.floor(x1 - pad_w), 0, self.frame_width - 1))
        lock_y1 = int(np.clip(np.floor(y1 - pad_h), 0, self.frame_height - 1))
        lock_x2 = int(np.clip(np.ceil(x2 + pad_w), lock_x1 + 1, self.frame_width))
        lock_y2 = int(np.clip(np.ceil(y2 + pad_h), lock_y1 + 1, self.frame_height))
        return lock_x1, lock_y1, lock_x2, lock_y2

    def _update_timing(self, criteria_met: bool, now: float):
        if criteria_met:
            if (
                self._last_criteria_met_time is not None
                and self._lock_start is not None
                and now - self._last_criteria_met_time > self._grace_time
            ):
                self._lock_start = now
                self._lock_duration = 0.0
                self._is_locked = False
                self._log.info("LOCK", "Lock streak restarted after grace timeout")

            if self._lock_start is None:
                self._lock_start = now
                self._lock_duration = 0.0
                self._is_locked = False
                self._log.info("LOCK", "Lock timer started")

            self._last_criteria_met_time = now
            self._lock_duration = max(now - self._lock_start, 0.0)
            return

        if self._last_criteria_met_time is not None:
            gap = now - self._last_criteria_met_time
            if gap <= self._grace_time:
                return

        if self._lock_start is not None and self._lock_duration > 0.0 and not self._is_locked:
            self._log.info(
                "LOCK",
                f"Lock broken duration={self._lock_duration:.2f}s",
            )

        self._lock_start = None
        self._last_criteria_met_time = None
        self._lock_duration = 0.0
        self._is_locked = False

    @staticmethod
    def _compute_coverage(
        target_bbox: np.ndarray,
        lx1: float, ly1: float, lx2: float, ly2: float,
    ) -> float:
        """Target bbox'in lock rect icindeki kapsama orani."""
        tx1, ty1, tx2, ty2 = target_bbox
        ix1 = max(tx1, lx1)
        iy1 = max(ty1, ly1)
        ix2 = min(tx2, lx2)
        iy2 = min(ty2, ly2)
        if ix2 <= ix1 or iy2 <= iy1:
            return 0.0
        inter_area = (ix2 - ix1) * (iy2 - iy1)
        target_area = max((tx2 - tx1) * (ty2 - ty1), 1.0)
        return inter_area / target_area
