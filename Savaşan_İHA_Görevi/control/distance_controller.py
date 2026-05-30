"""
Savaşan İHA - Distance Controller

Hedef İHA'ya olan mesafeyi bbox genişliği (width) üzerinden kontrol eder.
Bbox küçük → yaklaş (forward), büyük → uzaklaş (reverse).
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from core.logger import get_logger


class DistanceController:
    """
    Bbox genişliği (width) tabanlı mesafe kontrolü.

    width_ratio = bbox_width / frame_width
    desired_width_ratio → hedef mesafe (~70-100m arası)
    """

    def __init__(
        self,
        frame_width: int = 1280,
        frame_height: int = 720,
        fwd_kp: float = 3.0,
        fwd_ki: float = 0.1,
        fwd_kd: float = 1.0,
        max_forward_speed: float = 6.0,
        max_reverse_speed: float = 3.0,
        desired_width_ratio: float = 0.040,
        min_width_ratio: float = 0.015,
        max_width_ratio: float = 0.090,
        max_speed_delta: float = 0.8,
    ):
        self._log = get_logger()
        self._frame_width = float(frame_width)
        self._desired_ratio = desired_width_ratio
        self._min_ratio = min_width_ratio
        self._max_ratio = max_width_ratio
        self._max_fwd = max_forward_speed
        self._max_rev = max_reverse_speed
        self._kp = fwd_kp
        self._ki = fwd_ki
        self._kd = fwd_kd
        self._max_delta = max_speed_delta

        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_cmd = 0.0
        self._prev_ratio: Optional[float] = None

    def compute(
        self,
        bbox_width: float,
        x_error: float = 0.0,
        y_error: float = 0.0,
        dt: float = 0.033,
    ) -> float:
        """
        İleri hızı (forward speed) hedef bbox genişliğine göre hesaplar.

        Parametreler:
            bbox_width: hedef bbox genişliği (piksel)
            x_error: normalleştirilmiş yatay hata (hizalama kontrolü)
            dt: zaman adımı

        Dönen:
            forward_speed (m/s), pozitif = ileri, negatif = geri
        """
        ratio = bbox_width / max(self._frame_width, 1.0)
        prev_ratio = self._prev_ratio
        self._prev_ratio = ratio

        if ratio < 1e-6:
            self._prev_cmd = min(self._max_fwd * 0.6, 10.0)
            return self._prev_cmd

        log_error = np.log(self._desired_ratio / ratio)  # pozitif = çok uzak

        # PID
        p = self._kp * log_error
        self._integral += log_error * dt
        self._integral = np.clip(self._integral, -5.0, 5.0)
        i = self._ki * self._integral

        if dt > 1e-6:
            d = self._kd * (log_error - self._prev_error) / dt
        else:
            d = 0.0
        self._prev_error = log_error

        speed = p + i + d

        size_ratio = ratio / max(self._desired_ratio, 1e-6)
        edge_error = max(abs(x_error), abs(y_error))
        
        # Hız profilleri - hedefe uzaklığa göre
        if size_ratio < 0.30:
            speed = max(speed, self._max_fwd * 0.95)
        elif size_ratio < 0.55:
            speed = max(speed, self._max_fwd * 0.80)
        elif size_ratio < 0.85:
            speed = max(speed, self._max_fwd * 0.55)
        elif size_ratio < 1.05:
            speed = min(speed, self._max_fwd * 0.38)
        else:
            speed = min(speed, self._max_fwd * 0.22)

        # Hedef merkezden kaçıyorsa hızı düşür ki kamera hedeften kopmasın
        if edge_error > 0.75:
            speed *= 0.35
        elif edge_error > 0.50:
            speed *= 0.55
        elif edge_error > 0.30:
            speed *= 0.75

        # Hedef hızlı yaklaşıyorsa fren yap
        if prev_ratio is not None and prev_ratio > 1e-6:
            growth = ratio / prev_ratio
            if growth > 1.10 and size_ratio > 0.45:
                speed *= 0.72
            elif growth > 1.04 and size_ratio > 0.75:
                speed *= 0.82

        # Sınırla
        speed = float(np.clip(speed, -self._max_rev, self._max_fwd))

        # İvme sınırlaması (slew limiting)
        delta = speed - self._prev_cmd
        if abs(delta) > self._max_delta:
            speed = self._prev_cmd + self._max_delta * np.sign(delta)
        self._prev_cmd = speed

        # Log
        self._log.debug("DIST",
            f"FWD: speed={speed:+.1f}m/s bbox_w={bbox_width:.1f}px ratio={ratio:.6f} "
            f"log_err={log_error:+.2f} x_err={x_error:+.2f}",
            debounce=0.15)

        return speed

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_cmd = 0.0
        self._prev_ratio = None
