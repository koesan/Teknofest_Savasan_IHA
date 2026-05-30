"""
Savaşan İHA - Search Strategies

Hedef bulunamadığında arama kalıpları.
"""

from __future__ import annotations

import math
import time
from typing import Optional, Tuple

from core.logger import get_logger


class SearchStrategy:
    """
    Arama kalıpları jeneratörü.

    Kalıplar:
    - expanding_square: küçük kareyle başla, giderek büyüt
    - racetrack: oval pist
    - lawnmower: çim biçme deseni
    """

    def __init__(
        self,
        pattern: str = "expanding_square",
        start_leg: float = 30.0,
        leg_increment: float = 20.0,
        speed: float = 5.0,
        yaw_scan_rate: float = 30.0,
        max_radius: float = 200.0,
        center_lat: float = -35.3632622,
        center_lon: float = 149.1652376,
        altitude: float = 40.0,
        yaw_scan_enabled: bool = True,
    ):
        self._log = get_logger()
        self._pattern = pattern
        self._start_leg = start_leg
        self._leg_increment = leg_increment
        self._speed = speed
        self._yaw_scan_rate = yaw_scan_rate
        self._max_radius = max_radius
        self._center_lat = center_lat
        self._center_lon = center_lon
        self._altitude = altitude
        self._yaw_scan_enabled = yaw_scan_enabled

        # State
        self._current_leg = 0
        self._current_direction = 0  # 0=N, 1=E, 2=S, 3=W
        self._current_distance = start_leg
        self._legs_per_turn = 2  # iki bacak sonra mesafe artar
        self._leg_count_in_turn = 0
        self._last_wp_time: Optional[float] = None
        self._search_start_time: Optional[float] = None
        self._segment_started_at: Optional[float] = None
        self._current_waypoint: Optional[Tuple[float, float, float]] = None
        self._current_waypoint_index = 0
        
        # Yaw scan state
        self._yaw_scan_phase = 0  # 0=left, 1=center, 2=right, 3=center
        self._yaw_scan_start_time: Optional[float] = None

    def start(self):
        """Aramayı başlat/sıfırla."""
        self._current_leg = 0
        self._current_direction = 0
        self._current_distance = self._start_leg
        self._leg_count_in_turn = 0
        self._search_start_time = time.time()
        self._last_wp_time = None
        self._segment_started_at = None
        self._current_waypoint = None
        self._current_waypoint_index = 0
        self._relative_north = 0.0
        self._relative_east = 0.0
        self._log.info("MISSION", f"Search started: pattern={self._pattern}")

    def get_next_velocity(self, hunter_heading: float) -> Tuple[float, float, float, float]:
        """
        Arama kalıbı için bir sonraki hız komutu.

        Returns:
            (vx, vy, vz, yaw_rate) body frame
        """
        now = time.time()

        if self._pattern == "expanding_square":
            return self._expanding_square(hunter_heading, now)
        else:
            # Default: basit yaw taraması ile ileri uç
            return (self._speed, 0.0, 0.0, self._yaw_scan_rate)

    def _expanding_square(
        self, heading: float, now: float
    ) -> Tuple[float, float, float, float]:
        """
        Expanding square arama kalıbı.
        Her bacakta düz uç, sonra 90° sağa dön.
        İki bacak sonra mesafe artar.
        Yaw tarama ile kamera görüş alanını artır.
        """
        # Bacak süresi
        leg_time = self._current_distance / max(self._speed, 0.5)

        if self._last_wp_time is None:
            self._last_wp_time = now
            self._yaw_scan_start_time = now
            self._log.info("MISSION", f"SEARCH: starting leg 0 dist={self._current_distance:.0f}m")

        elapsed = now - self._last_wp_time

        if elapsed < leg_time:
            # Düz uçuş - yaw tarama ile
            yaw_rate = 0.0
            if self._yaw_scan_enabled:
                yaw_rate = self._compute_yaw_scan(now)
            self._log.debug("MISSION",
                f"SEARCH: flying leg={self._current_leg} dir={self._current_direction} "
                f"dist={self._current_distance:.0f}m elapsed={elapsed:.1f}s/{leg_time:.1f}s yaw_scan={yaw_rate:+.0f}°/s",
                debounce=0.5)
            return (self._speed, 0.0, 0.0, yaw_rate)
        else:
            # Bacak sonu - yeni bacak
            self._last_wp_time = now
            self._yaw_scan_start_time = now
            self._current_leg += 1
            self._leg_count_in_turn += 1

            if self._leg_count_in_turn >= self._legs_per_turn:
                self._current_distance += self._leg_increment
                self._leg_count_in_turn = 0

            if self._current_distance > self._max_radius:
                # Sıfırla
                self._current_distance = self._start_leg

            self._current_direction = (self._current_direction + 1) % 4

            self._log.info("MISSION",
                f"Search leg {self._current_leg}: "
                f"dir={self._current_direction} "
                f"dist={self._current_distance:.0f}m")

            # Dönüş komutu (yaw 90°)
            return (0.0, 0.0, 0.0, 90.0)

    def _compute_yaw_scan(self, now: float) -> float:
        """
        Yaw tarama hareketi hesapla.
        Sol-sağ tarama ile görüş alanını genişlet.
        """
        if self._yaw_scan_start_time is None:
            self._yaw_scan_start_time = now
            return 0.0
        
        elapsed = now - self._yaw_scan_start_time
        scan_period = 4.0  # 4 saniyede bir tam tarama döngüsü
        
        # Sinüzoidal tarama: -yaw_scan_rate ile +yaw_scan_rate arasında
        phase = (elapsed % scan_period) / scan_period  # 0-1
        # Sinüzoidal: 0->0.5 artış, 0.5->1 azalış
        if phase < 0.5:
            # Sağa tarama
            return self._yaw_scan_rate
        else:
            # Sola tarama
            return -self._yaw_scan_rate

    def get_goto_waypoint(
        self,
        hunter_lat: float,
        hunter_lon: float,
        hunter_heading: float,
    ) -> Optional[Tuple[float, float, float]]:
        """
        Arama noktası hesapla (GPS waypoint olarak).

        Returns:
            (lat, lon, alt) or None
        """
        heading_rad = math.radians(
            [0, 90, 180, 270][self._current_direction]
        )

        # NED offset
        north = self._current_distance * math.cos(heading_rad)
        east = self._current_distance * math.sin(heading_rad)

        # GPS offset
        lat = self._center_lat + north / 111111.0
        lon = self._center_lon + east / (111111.0 * math.cos(math.radians(self._center_lat)))

        return (lat, lon, self._altitude)

    def get_next_search_waypoint(
        self,
        hunter_lat: float,
        hunter_lon: float,
        now: Optional[float] = None,
        reach_radius_m: float = 18.0,
    ) -> Tuple[float, float, float]:
        """
        Fixed-wing arama için bir sonraki waypoint'i üret.

        Waypoint, hedefe yaklaşıldığında veya segment süresi dolduğunda ilerletilir.
        """
        now = now or time.time()
        if self._pattern != "expanding_square":
            return self._orbit_waypoint(now)

        if self._current_waypoint is None:
            self._segment_started_at = now
            self._current_waypoint = self._build_expanding_square_waypoint()
            self._log.info(
                "MISSION",
                f"SEARCH WP #{self._current_waypoint_index}: "
                f"lat={self._current_waypoint[0]:.7f} lon={self._current_waypoint[1]:.7f} "
                f"alt={self._current_waypoint[2]:.1f}m leg={self._current_leg}",
            )
            return self._current_waypoint

        distance = self._distance_m(
            hunter_lat,
            hunter_lon,
            self._current_waypoint[0],
            self._current_waypoint[1],
        )
        leg_timeout = max(self._current_distance / max(self._speed, 1.0), 4.0) * 1.6
        segment_age = now - (self._segment_started_at or now)

        if distance <= reach_radius_m or segment_age >= leg_timeout:
            reason = "reached" if distance <= reach_radius_m else "timeout"
            self._advance_square_leg()
            self._segment_started_at = now
            self._current_waypoint = self._build_expanding_square_waypoint()
            self._log.info(
                "MISSION",
                f"SEARCH NEXT ({reason}) WP #{self._current_waypoint_index}: "
                f"lat={self._current_waypoint[0]:.7f} lon={self._current_waypoint[1]:.7f} "
                f"dist={self._current_distance:.0f}m "
                f"dir={self._current_direction}",
            )

        return self._current_waypoint

    def _build_expanding_square_waypoint(self) -> Tuple[float, float, float]:
        heading_rad = math.radians([0, 90, 180, 270][self._current_direction])
        target_north = self._relative_north + self._current_distance * math.cos(heading_rad)
        target_east = self._relative_east + self._current_distance * math.sin(heading_rad)
        lat = self._center_lat + target_north / 111111.0
        lon = self._center_lon + target_east / (111111.0 * math.cos(math.radians(self._center_lat)))
        self._current_waypoint_index += 1
        return (lat, lon, self._altitude)

    def _advance_square_leg(self):
        heading_rad = math.radians([0, 90, 180, 270][self._current_direction])
        self._relative_north += self._current_distance * math.cos(heading_rad)
        self._relative_east += self._current_distance * math.sin(heading_rad)

        self._current_leg += 1
        self._leg_count_in_turn += 1
        if self._leg_count_in_turn >= self._legs_per_turn:
            self._current_distance += self._leg_increment
            self._leg_count_in_turn = 0
        if math.hypot(self._relative_north, self._relative_east) > self._max_radius:
            self._current_distance = self._start_leg
            self._relative_north = 0.0
            self._relative_east = 0.0
        self._current_direction = (self._current_direction + 1) % 4

    def _orbit_waypoint(self, now: float) -> Tuple[float, float, float]:
        angle_deg = ((now - (self._search_start_time or now)) * max(self._yaw_scan_rate, 15.0)) % 360.0
        angle_rad = math.radians(angle_deg)
        north = self._max_radius * math.cos(angle_rad)
        east = self._max_radius * math.sin(angle_rad)
        lat = self._center_lat + north / 111111.0
        lon = self._center_lon + east / (111111.0 * math.cos(math.radians(self._center_lat)))
        return (lat, lon, self._altitude)

    @staticmethod
    def _distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        radius = 6371000.0
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        alat1 = math.radians(lat1)
        alat2 = math.radians(lat2)
        a = math.sin(dlat / 2) ** 2 + math.cos(alat1) * math.cos(alat2) * math.sin(dlon / 2) ** 2
        return radius * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
