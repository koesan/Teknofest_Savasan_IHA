from __future__ import annotations

import math
import threading
import time
from typing import Optional, Tuple

import numpy as np

from logger import get_logger

try:
    from dronekit import LocationGlobalRelative, VehicleMode, connect
    from pymavlink import mavutil
    DRONEKIT_AVAILABLE = True
except ImportError:
    DRONEKIT_AVAILABLE = False


class Vehicle:
    def __init__(self, connection_string: str, connection_timeout: int, arm_timeout: int):
        self._log = get_logger()
        self._connection_string = connection_string
        self._connection_timeout = connection_timeout
        self._arm_timeout = arm_timeout
        self._vehicle = None
        self._lock = threading.Lock()
        self._home_position: Optional[dict] = None
        self._last_guided_command = 0.0
        self._last_fw_command_time = 0.0
        self._last_fw_signature: Optional[Tuple[float, float, float, float]] = None
        self._last_airspeed: Optional[float] = None
        self._last_vtol_transition = 0.0
        self._last_takeoff_command = 0.0

    @property
    def connected(self) -> bool:
        return self._vehicle is not None

    def connect(self) -> bool:
        if not DRONEKIT_AVAILABLE:
            self._log.error("VEHICLE", "dronekit not installed")
            return False
        try:
            self._log.info("VEHICLE", f"connecting: {self._connection_string}")
            self._vehicle = connect(self._connection_string, wait_ready=True, timeout=self._connection_timeout)
            self._log.info("VEHICLE", f"connected mode={self._vehicle.mode.name} armed={self._vehicle.armed}")
            return True
        except Exception as exc:
            self._log.error("VEHICLE", f"connect failed: {exc}")
            return False

    def configure_for_mission(
        self,
        disable_arming_checks: bool,
        disable_compass: bool,
        airframe_profile: Optional[str],
        reboot_if_changed: bool,
    ) -> bool:
        del airframe_profile, reboot_if_changed
        self.set_parameter("Q_GUIDED_MODE", 1)
        if disable_arming_checks:
            self.set_parameter("ARMING_CHECK", 0)
        if disable_compass:
            for name in ("COMPASS_USE", "COMPASS_USE2", "COMPASS_USE3"):
                self.set_parameter(name, 0)
        return True

    def set_parameter(self, name: str, value) -> bool:
        if not self.connected:
            return False
        try:
            self._vehicle.parameters[name] = value
            self._log.info("VEHICLE", f"param {name}={value}")
            return True
        except Exception as exc:
            self._log.warn("VEHICLE", f"param set failed {name}: {exc}")
            return False

    def set_mode(self, mode_name: str, timeout: float = 12.0) -> bool:
        if not self.connected:
            return False
        self._vehicle.mode = VehicleMode(mode_name)
        deadline = time.time() + timeout
        while time.time() < deadline:
            current = getattr(self._vehicle.mode, "name", "")
            if str(current).upper() == mode_name.upper():
                self._log.info("VEHICLE", f"mode -> {mode_name}")
                return True
            time.sleep(0.2)
        self._log.warn("VEHICLE", f"mode timeout expected={mode_name} current={self._vehicle.mode}")
        return False

    def _supported_mode_names(self) -> set:
        if not self.connected:
            return set()
        try:
            modes = getattr(self._vehicle, "_mode_mapping", None) or {}
            return {str(name).upper() for name in modes.keys()}
        except Exception:
            return set()

    def choose_hover_mode(self) -> str:
        modes = self._supported_mode_names()
        for mode_name in ("QLOITER", "QHOVER", "QSTABILIZE", "GUIDED"):
            if not modes or mode_name in modes:
                return mode_name
        return "GUIDED"

    def _wait_armable(self, timeout: float) -> bool:
        if not self.connected:
            return False
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                if bool(getattr(self._vehicle, "is_armable", False)):
                    return True
            except Exception:
                pass
            time.sleep(1.0)
        self._log.warn("VEHICLE", "armable timeout, forcing continue")
        return True

    def _force_arm(self) -> bool:
        if not self.connected:
            return False
        try:
            self._vehicle.armed = True
            time.sleep(2.0)
            if self._vehicle.armed:
                return True
        except Exception:
            pass
        try:
            msg = self._vehicle.message_factory.command_long_encode(
                0, 0, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0
            )
            with self._lock:
                self._vehicle.send_mavlink(msg)
                if hasattr(self._vehicle, "flush"):
                    self._vehicle.flush()
            time.sleep(2.0)
            return bool(self._vehicle.armed)
        except Exception as exc:
            self._log.error("VEHICLE", f"force arm failed: {exc}")
            return False

    def arm_and_takeoff(self, target_altitude_m: float, completion_ratio: float = 0.98) -> bool:
        if not self.connected:
            return False
        hover_mode = self.choose_hover_mode()
        self.set_mode(hover_mode)
        self._wait_armable(timeout=max(float(self._arm_timeout), 60.0))
        if not self._force_arm():
            self._log.error("VEHICLE", "arming failed")
            return False

        self.set_mode("GUIDED")
        self._save_home_position()
        self._log.info("VEHICLE", f"takeoff to {target_altitude_m:.1f} m")
        msg = self._vehicle.message_factory.command_long_encode(
            0, 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, target_altitude_m
        )
        with self._lock:
            self._vehicle.send_mavlink(msg)
            if hasattr(self._vehicle, "flush"):
                self._vehicle.flush()
        completion_ratio = float(np.clip(completion_ratio, 0.5, 1.0))
        deadline = time.time() + 180.0
        while time.time() < deadline:
            alt = float(self._vehicle.location.global_relative_frame.alt or 0.0)
            if alt >= target_altitude_m * completion_ratio:
                self._log.info("VEHICLE", f"takeoff complete alt={alt:.1f} m")
                return True
            time.sleep(0.5)
        self._log.error("VEHICLE", "takeoff timeout")
        return False

    def _save_home_position(self):
        loc = self._vehicle.location.global_relative_frame
        self._home_position = {"lat": float(loc.lat or 0.0), "lon": float(loc.lon or 0.0), "alt": 0.0}
        self._log.info(
            "VEHICLE",
            f"target/home saved lat={self._home_position['lat']:.7f} lon={self._home_position['lon']:.7f}",
        )

    def get_home_position(self) -> Optional[dict]:
        return self._home_position

    def transition_to_fw(self, wait_time_s: float, throttle_pwm: int, airspeed: float = 18.0) -> bool:
        if not self.connected:
            return False
        try:
            self.set_mode("FBWA")
            self.set_rc_overrides(throttle_pwm=throttle_pwm)
            time.sleep(max(wait_time_s, 1.0))
            self.set_rc_overrides(roll_pwm=1500, pitch_pwm=1500, throttle_pwm=throttle_pwm, yaw_pwm=1500)
            self._log.info("VEHICLE", f"fixed-wing cruise ready in FBWA throttle={throttle_pwm}")
            return True
        except Exception as exc:
            self._log.error("VEHICLE", f"fw transition failed: {exc}")
            return False

    def transition_to_vtol(self, hover_mode: Optional[str] = None, timeout: float = 12.0) -> bool:
        if not self.connected:
            return False
        if hover_mode is None:
            hover_mode = self.choose_hover_mode()
        now = time.time()
        if now - self._last_vtol_transition < 2.0:
            return True
        self._last_vtol_transition = now
        try:
            msg = self._vehicle.message_factory.command_long_encode(
                0, 0, mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION, 0, 3, 0, 0, 0, 0, 0, 0
            )
            with self._lock:
                self._vehicle.send_mavlink(msg)
                if hasattr(self._vehicle, "flush"):
                    self._vehicle.flush()
            time.sleep(2.0)
        except Exception as exc:
            self._log.warn("VEHICLE", f"vtol transition command failed: {exc}")
        self.clear_rc_overrides()
        ok = self.set_mode(hover_mode, timeout=timeout)
        if ok:
            self._log.info("VEHICLE", f"vtol mode active: {hover_mode}")
        return ok

    def transition_to_guided_vtol(self, target_altitude_m: Optional[float] = None) -> bool:
        if not self.connected:
            return False
        hover_mode = self.choose_hover_mode()
        ok = self.transition_to_vtol(hover_mode=hover_mode, timeout=8.0)
        if not ok:
            return False
        guided_ok = self.set_mode("GUIDED", timeout=8.0)
        if guided_ok and target_altitude_m is not None:
            self.command_vtol_takeoff(target_altitude_m, min_interval_s=0.0)
        return guided_ok

    def command_vtol_takeoff(self, target_altitude_m: float, min_interval_s: float = 3.0) -> bool:
        if not self.connected:
            return False
        now = time.time()
        if now - self._last_takeoff_command < min_interval_s:
            return False
        self._last_takeoff_command = now
        try:
            msg = self._vehicle.message_factory.command_long_encode(
                0, 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, target_altitude_m
            )
            with self._lock:
                self._vehicle.send_mavlink(msg)
                if hasattr(self._vehicle, "flush"):
                    self._vehicle.flush()
            self._log.info("VEHICLE", f"vtol climb command target={target_altitude_m:.1f} m")
            return True
        except Exception as exc:
            self._log.warn("VEHICLE", f"vtol climb command failed: {exc}")
            return False

    def set_airspeed(self, airspeed: float):
        if not self.connected:
            return
        try:
            self._vehicle.airspeed = airspeed
            if self._last_airspeed is None or abs(self._last_airspeed - airspeed) >= 0.5:
                self._log.info("VEHICLE", f"airspeed -> {airspeed:.1f} m/s")
                self._last_airspeed = airspeed
        except Exception:
            pass

    def goto(self, lat: float, lon: float, alt: float, airspeed: Optional[float] = None):
        if not self.connected:
            return
        point = LocationGlobalRelative(lat, lon, alt)
        with self._lock:
            if airspeed is None:
                self._vehicle.simple_goto(point)
            else:
                self._vehicle.simple_goto(point, airspeed=airspeed)

    def command_guided_waypoint(self, lat: float, lon: float, alt: float, airspeed: float, min_interval_s: float):
        now = time.time()
        if now - self._last_guided_command < min_interval_s:
            return
        self._last_guided_command = now
        self.set_airspeed(airspeed)
        self.goto(lat, lon, alt, airspeed=airspeed)

    def command_fixed_wing_heading(
        self,
        target_heading_deg: float,
        target_alt: float,
        airspeed: float,
        lookahead_time: float = 3.0,
        min_command_interval: float = 0.8,
        refresh_interval: float = 3.5,
        heading_tolerance_deg: float = 5.0,
        alt_tolerance_m: float = 1.0,
    ) -> bool:
        if not self.connected:
            return False

        now = time.time()
        telem = self.get_telemetry()
        heading = float(telem["heading"] or 0.0)
        current_alt = float(telem["alt"] or target_alt)
        target_heading_deg = target_heading_deg % 360.0
        delta = ((target_heading_deg - heading + 540.0) % 360.0) - 180.0

        signature = (
            round(target_heading_deg, 1),
            round(float(target_alt), 1),
            round(float(airspeed), 1),
            round(float(lookahead_time), 1),
        )
        last_dt = now - self._last_fw_command_time
        if self._last_fw_signature == signature and last_dt < refresh_interval:
            if abs(delta) < heading_tolerance_deg and abs(current_alt - target_alt) < alt_tolerance_m:
                return False
        if last_dt < min_command_interval:
            return False

        travel_distance = max(float(airspeed), 5.0) * max(float(lookahead_time), 1.0)
        heading_rad = math.radians(target_heading_deg)
        north = travel_distance * math.cos(heading_rad)
        east = travel_distance * math.sin(heading_rad)
        lat = float(telem["lat"] or 0.0)
        lon = float(telem["lon"] or 0.0)
        target_lat, target_lon = self.offset_position(lat, lon, north, east)

        self.set_airspeed(airspeed)
        self.goto(target_lat, target_lon, target_alt, airspeed=airspeed)
        self._last_fw_command_time = now
        self._last_fw_signature = signature
        self._log.info(
            "VEHICLE",
            f"fw hold hdg={target_heading_deg:.0f} alt={target_alt:.1f} spd={airspeed:.1f} lookahead={travel_distance:.0f}m",
            debounce=0.2,
        )
        return True

    def command_fixed_wing_attitude(
        self,
        target_heading_deg: float,
        target_alt: float,
        cruise_throttle: int = 1650,
        min_throttle: int = 1500,
        max_throttle: int = 1850,
        min_command_interval: float = 0.25,
        roll_limit_pwm: int = 140,
        pitch_limit_pwm: int = 80,
        turn_gain: float = 0.75,
        alt_gain_divisor: float = 14.0,
        pitch_bias_pwm: int = 0,
        invert_pitch: bool = False,
        invert_roll: bool = False,
    ) -> bool:
        if not self.connected:
            return False
        now = time.time()
        if now - self._last_fw_command_time < min_command_interval:
            return False

        telem = self.get_telemetry()
        current_heading = float(telem["heading"] or 0.0)
        current_alt = float(telem["alt"] or target_alt)
        current_speed = float(telem["groundspeed"] or telem["airspeed"] or 0.0)
        heading_error = ((target_heading_deg - current_heading + 540.0) % 360.0) - 180.0
        alt_error = float(target_alt - current_alt)

        roll_sign = -1.0 if invert_roll else 1.0
        pitch_sign = -1.0 if invert_pitch else 1.0
        roll_pwm = int(1500 + roll_sign * np.tanh(math.radians(heading_error) * turn_gain) * float(roll_limit_pwm))
        roll_pwm = int(np.clip(roll_pwm, 1500 - roll_limit_pwm, 1500 + roll_limit_pwm))
        pitch_pwm = int(
            1500
            + pitch_sign * np.tanh(alt_error / max(alt_gain_divisor, 1.0)) * float(pitch_limit_pwm)
            + pitch_sign * int(pitch_bias_pwm)
        )
        pitch_pwm = int(np.clip(pitch_pwm, 1500 - pitch_limit_pwm, 1500 + pitch_limit_pwm))
        speed_error = max(0.0, 12.0 - current_speed)
        turn_boost = abs(heading_error) * 0.6
        throttle_pwm = int(
            np.clip(
                cruise_throttle + speed_error * 12.0 + turn_boost,
                min_throttle,
                max_throttle,
            )
        )

        self.set_rc_overrides(
            roll_pwm=roll_pwm,
            pitch_pwm=pitch_pwm,
            throttle_pwm=throttle_pwm,
            yaw_pwm=1500,
        )
        self._log.info(
            "VEHICLE",
            f"fw attitude hdg={current_heading:.0f}->{target_heading_deg:.0f} err={heading_error:+.0f} alt={current_alt:.1f}->{target_alt:.1f} bias={pitch_bias_pwm:+d} rc=({roll_pwm},{pitch_pwm},{throttle_pwm})",
            debounce=0.15,
        )
        self._last_fw_command_time = now
        return True

    def command_fixed_wing_neutral(
        self,
        throttle_pwm: int,
        roll_pwm: int = 1500,
        pitch_pwm: int = 1500,
        yaw_pwm: int = 1500,
        min_command_interval: float = 0.25,
    ) -> bool:
        if not self.connected:
            return False
        now = time.time()
        if now - self._last_fw_command_time < min_command_interval:
            return False

        self.set_rc_overrides(
            roll_pwm=roll_pwm,
            pitch_pwm=pitch_pwm,
            throttle_pwm=throttle_pwm,
            yaw_pwm=yaw_pwm,
        )
        self._last_fw_command_time = now
        self._log.info(
            "VEHICLE",
            f"fw neutral rc=({roll_pwm},{pitch_pwm},{throttle_pwm},{yaw_pwm})",
            debounce=0.2,
        )
        return True

    def set_rc_overrides(
        self,
        roll_pwm: Optional[int] = None,
        pitch_pwm: Optional[int] = None,
        throttle_pwm: Optional[int] = None,
        yaw_pwm: Optional[int] = None,
    ):
        if not self.connected:
            return
        current = dict(getattr(self._vehicle.channels, "overrides", {}) or {})
        mapping = {"1": roll_pwm, "2": pitch_pwm, "3": throttle_pwm, "4": yaw_pwm}
        for channel, value in mapping.items():
            if value is None:
                current.pop(channel, None)
            else:
                current[channel] = int(value)
        self._vehicle.channels.overrides = current

    def clear_rc_overrides(self):
        if self.connected:
            self._vehicle.channels.overrides = {}

    def get_telemetry(self) -> dict:
        if not self.connected:
            return {
                "lat": 0.0,
                "lon": 0.0,
                "alt": 0.0,
                "heading": 0.0,
                "airspeed": 0.0,
                "groundspeed": 0.0,
                "mode": "UNKNOWN",
                "armed": False,
            }
        loc = self._vehicle.location.global_relative_frame
        return {
            "lat": float(loc.lat or 0.0),
            "lon": float(loc.lon or 0.0),
            "alt": float(loc.alt or 0.0),
            "heading": float(self._vehicle.heading or 0.0),
            "airspeed": float(self._vehicle.airspeed or 0.0),
            "groundspeed": float(self._vehicle.groundspeed or 0.0),
            "mode": str(self._vehicle.mode.name),
            "armed": bool(self._vehicle.armed),
        }

    def safe_land(self):
        if not self.connected:
            return
        try:
            msg = self._vehicle.message_factory.command_long_encode(
                0, 0, mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION, 0, 3, 0, 0, 0, 0, 0, 0
            )
            with self._lock:
                self._vehicle.send_mavlink(msg)
                if hasattr(self._vehicle, "flush"):
                    self._vehicle.flush()
            time.sleep(4.0)
            self.set_mode("QLAND")
        except Exception:
            self.set_mode("LAND")

    def close(self):
        if self._vehicle is not None:
            try:
                self._vehicle.close()
            except Exception:
                pass
            self._vehicle = None

    @staticmethod
    def distance_between(loc1: dict, loc2: dict) -> float:
        return _distance_between(loc1["lat"], loc1["lon"], loc2["lat"], loc2["lon"])

    @staticmethod
    def bearing_to(loc1: dict, loc2: dict) -> float:
        return _bearing_between(loc1["lat"], loc1["lon"], loc2["lat"], loc2["lon"])

    @staticmethod
    def offset_position(lat: float, lon: float, north_m: float, east_m: float) -> Tuple[float, float]:
        new_lat = lat + north_m / 111111.0
        scale = 111111.0 * math.cos(math.radians(lat))
        new_lon = lon + east_m / max(scale, 1.0)
        return new_lat, new_lon


def _distance_between(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    radius = 6371000.0
    rlat1 = math.radians(lat1)
    rlon1 = math.radians(lon1)
    rlat2 = math.radians(lat2)
    rlon2 = math.radians(lon2)
    dlat = rlat2 - rlat1
    dlon = rlon2 - rlon1
    a = math.sin(dlat / 2.0) ** 2 + math.cos(rlat1) * math.cos(rlat2) * math.sin(dlon / 2.0) ** 2
    return radius * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


def _bearing_between(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    rlat1 = math.radians(lat1)
    rlon1 = math.radians(lon1)
    rlat2 = math.radians(lat2)
    rlon2 = math.radians(lon2)
    dlon = rlon2 - rlon1
    x = math.sin(dlon) * math.cos(rlat2)
    y = math.cos(rlat1) * math.sin(rlat2) - math.sin(rlat1) * math.cos(rlat2) * math.cos(dlon)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0
