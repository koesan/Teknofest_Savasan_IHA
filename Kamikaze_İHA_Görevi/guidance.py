from __future__ import annotations

import math
from typing import Tuple


class DiveGuidance:
    def __init__(
        self,
        target_lat: float,
        target_lon: float,
        cruise_altitude_m: float,
        minimum_altitude_m: float,
        dive_angle_deg: float,
        pullup_target_altitude_m: float,
        extra_margin_m: float,
        approach_distance_m: float,
        transition_lead_distance_m: float,
        target_offset_m: float = 8.0,
        effective_dive_angle_deg: float = 18.0,
    ):
        self.target_lat = target_lat
        self.target_lon = target_lon
        self.cruise_altitude_m = cruise_altitude_m
        self.minimum_altitude_m = minimum_altitude_m
        self.dive_angle_deg = dive_angle_deg
        self.pullup_target_altitude_m = pullup_target_altitude_m
        self.extra_margin_m = extra_margin_m
        self.approach_distance_m = approach_distance_m
        self.transition_lead_distance_m = transition_lead_distance_m
        self.target_offset_m = target_offset_m
        self.effective_dive_angle_deg = effective_dive_angle_deg

        angle_rad = math.radians(max(effective_dive_angle_deg, 1.0))
        vertical_drop = max(cruise_altitude_m - minimum_altitude_m, 1.0)
        self.dive_start_distance_m = target_offset_m + (vertical_drop / math.tan(angle_rad)) + extra_margin_m
        self.fixed_wing_entry_distance_m = self.dive_start_distance_m + max(transition_lead_distance_m, 0.0)
        self.total_attack_run_distance_m = self.dive_start_distance_m + max(approach_distance_m, 0.0)

    def should_start_dive(self, distance_to_target_m: float, altitude_m: float, min_start_alt_m: float) -> bool:
        # İHA'nın dalışa başlaması gerekip gerekmediğini kontrol et
        return altitude_m >= (min_start_alt_m - 5.0) and distance_to_target_m <= self.dive_start_distance_m

    def should_begin_fixed_wing_approach(self, distance_to_target_m: float, altitude_m: float, min_start_alt_m: float) -> bool:
        return altitude_m >= min_start_alt_m and distance_to_target_m <= self.fixed_wing_entry_distance_m

    def compute_approach_waypoint(
        self,
        current_lat: float,
        current_lon: float,
        step_distance_m: float,
    ) -> Tuple[float, float, float]:
        # Seyir irtifasında hedefe doğru düz uçuş koordinatı hesaplar.
        bearing = self.bearing_between(current_lat, current_lon, self.target_lat, self.target_lon)
        north = step_distance_m * math.cos(math.radians(bearing))
        east = step_distance_m * math.sin(math.radians(bearing))
        next_lat, next_lon = self.offset_position(current_lat, current_lon, north, east)
        return next_lat, next_lon, self.cruise_altitude_m

    def compute_dive_waypoint(
        self,
        current_lat: float,
        current_lon: float,
        current_alt_m: float,
        step_distance_m: float,
    ) -> Tuple[float, float, float]:
        # Hedefe süzülüş açısıyla yaklaşmak için dalış koordinatı hesaplar.
        bearing = self.bearing_between(current_lat, current_lon, self.target_lat, self.target_lon)
        north = step_distance_m * math.cos(math.radians(bearing))
        east = step_distance_m * math.sin(math.radians(bearing))
        next_lat, next_lon = self.offset_position(current_lat, current_lon, north, east)
        descent = step_distance_m * math.tan(math.radians(self.dive_angle_deg))
        next_alt = max(
            current_alt_m - descent,
            self.minimum_altitude_m,
        )
        return next_lat, next_lon, next_alt

    def compute_pullup_waypoint(
        self,
        current_lat: float,
        current_lon: float,
        current_alt_m: float,
        current_heading_deg: float,
        step_distance_m: float,
    ) -> Tuple[float, float, float]:
        # QR okunduktan sonra mevcut yönde tırmanış koordinatı hesaplar.
        north = step_distance_m * math.cos(math.radians(current_heading_deg))
        east = step_distance_m * math.sin(math.radians(current_heading_deg))
        next_lat, next_lon = self.offset_position(current_lat, current_lon, north, east)
        climb = step_distance_m * math.tan(math.radians(self.dive_angle_deg))
        next_alt = min(
            current_alt_m + climb,
            self.pullup_target_altitude_m,
        )
        return next_lat, next_lon, next_alt

    def altitude_for_distance(self, distance_to_target_m: float) -> float:
        # Süzülüş hattındaki ideal irtifayı hesaplar.
        angle_rad = math.radians(max(self.dive_angle_deg, 1.0))
        ideal_alt = distance_to_target_m * math.tan(angle_rad) + self.minimum_altitude_m
        return min(ideal_alt, self.cruise_altitude_m)

    @staticmethod
    def distance_between(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        radius = 6371000.0
        rlat1 = math.radians(lat1)
        rlon1 = math.radians(lon1)
        rlat2 = math.radians(lat2)
        rlon2 = math.radians(lon2)
        dlat = rlat2 - rlat1
        dlon = rlon2 - rlon1
        a = math.sin(dlat / 2.0) ** 2 + math.cos(rlat1) * math.cos(rlat2) * math.sin(dlon / 2.0) ** 2
        return radius * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))

    @staticmethod
    def bearing_between(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        rlat1 = math.radians(lat1)
        rlon1 = math.radians(lon1)
        rlat2 = math.radians(lat2)
        rlon2 = math.radians(lon2)
        dlon = rlon2 - rlon1
        x = math.sin(dlon) * math.cos(rlat2)
        y = math.cos(rlat1) * math.sin(rlat2) - math.sin(rlat1) * math.cos(rlat2) * math.cos(dlon)
        return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0

    @staticmethod
    def offset_position(lat: float, lon: float, north_m: float, east_m: float) -> Tuple[float, float]:
        new_lat = lat + north_m / 111111.0
        scale = 111111.0 * math.cos(math.radians(lat))
        new_lon = lon + east_m / max(scale, 1.0)
        return new_lat, new_lon
