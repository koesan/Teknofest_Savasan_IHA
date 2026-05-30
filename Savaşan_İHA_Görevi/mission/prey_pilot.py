"""
Savaşan İHA - Prey Autopilot

Av İHA otopilotu: sabit bir merkez etrafında saat yönünde
kararlı dairesel uçuş (native loiter).
"""

from __future__ import annotations

import time
from typing import Optional

from core.logger import get_logger


class PreyPilot:
    """
    Av İHA için basit patrol otopilotu.
    ArduPilot'un native loiter komutunu kullanarak sabit bir merkez
    etrafında saat yönünde pürüzsüz daire çizer.
    """

    def __init__(
        self,
        center_lat: float = -35.3632622,
        center_lon: float = 149.1652376,
        patrol_radius: float = 100.0,
        altitude: float = 100.0,
        min_altitude: float = 100.0,
        hold_altitude: bool = True,
        # Aşağıdaki parametreler geriye uyumluluk için kabul edilir ama kullanılmaz
        step_deg: float = 5.0,
        period: float = 1.0,
        fw_throttle: int = 1600,
    ):
        self._log = get_logger()
        self._center_lat = center_lat
        self._center_lon = center_lon
        self._radius = abs(patrol_radius)  # Pozitif = saat yönü (ArduPilot)
        self._altitude = altitude
        self._min_altitude = min_altitude
        self._hold_altitude = hold_altitude

        self._last_cmd_time: Optional[float] = None
        self._cmd_count = 0
        self._refresh_interval = 5.0  # Komutu tazeleme süresi (saniye)

    def update(self, vehicle, now: Optional[float] = None):
        """
        Av İHA devriye güncellemesi. Belirlenen merkez etrafında saat yönünde
        daire çizmesi için native loiter komutunu periyodik olarak gönderir.

        Parametreler:
            vehicle: Araç nesnesi (av)
            now: zaman damgası
        """
        now = now or time.time()

        if self._last_cmd_time is None:
            self._last_cmd_time = 0.0

        # Periyodik olarak komutu tazele
        if now - self._last_cmd_time >= self._refresh_interval:
            target_alt = self._altitude if self._hold_altitude else self._min_altitude

            if hasattr(vehicle, "loiter_at"):
                # Pozitif radius = saat yönünde dönüş (ArduPilot standardı)
                vehicle.loiter_at(
                    lat=self._center_lat,
                    lon=self._center_lon,
                    alt=target_alt,
                    radius=self._radius,
                )
            else:
                # Fallback: basit goto
                vehicle.goto(self._center_lat, self._center_lon, target_alt)

            self._last_cmd_time = now
            self._cmd_count += 1

            self._log.info(
                "MISSION",
                f"AV orbit #{self._cmd_count}: saat yönü loiter "
                f"merkez=({self._center_lat:.7f},{self._center_lon:.7f}) "
                f"alt={target_alt:.0f}m yarıçap={self._radius:.0f}m",
                debounce=1.0,
            )
