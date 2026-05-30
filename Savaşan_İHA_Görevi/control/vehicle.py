"""
Savaşan İHA - MAVLink Vehicle Abstraction

Dronekit/MAVLink tabanlı araç kontrolü.
Body frame velocity, takeoff, land, goto komutları.
"""

from __future__ import annotations

import math
import time
import threading
from typing import Optional, Tuple

import numpy as np

from core.logger import get_logger

try:
    from dronekit import connect, VehicleMode, LocationGlobalRelative
    from pymavlink import mavutil
    DRONEKIT_AVAILABLE = True
except ImportError:
    DRONEKIT_AVAILABLE = False


class Vehicle:
    """
    MAVLink araç soyutlaması.
    Thread-safe, connection monitoring dahil.
    """

    def __init__(
        self,
        connection_string: str,
        name: str = "UAV",
        connection_timeout: int = 60,
        arm_timeout: int = 60,
    ):
        self._log = get_logger()
        self._name = name
        self._conn_str = connection_string
        self._conn_timeout = connection_timeout
        self._arm_timeout = arm_timeout
        self._vehicle = None
        self._lock = threading.Lock()
        self._connected = False
        self._last_goto_time = 0.0
        self._last_airspeed: Optional[float] = None
        self._last_fw_command_time = 0.0
        self._last_fw_signature: Optional[Tuple[float, float, float, float]] = None
        self._landing_started = False
        self._last_rc_signature: Optional[Tuple[int, int, int, int]] = None

    @property
    def connected(self) -> bool:
        return self._connected and self._vehicle is not None

    def connect(self) -> bool:
        """Araca bağlan."""
        if not DRONEKIT_AVAILABLE:
            self._log.error("VEHICLE", "dronekit not installed!")
            return False

        try:
            self._log.info("VEHICLE", f"[{self._name}] Connecting to {self._conn_str}...")
            self._vehicle = connect(
                self._conn_str,
                wait_ready=True,
                timeout=self._conn_timeout,
            )
            self._connected = True
            self._landing_started = False
            self._log.info("VEHICLE",
                f"[{self._name}] Connected. Mode={self._vehicle.mode.name} "
                f"Armed={self._vehicle.armed}")
            try:
                self._vehicle.parameters['Q_ASSIST_SPEED'] = 30.0
                self._vehicle.parameters['Q_ASSIST_ALT'] = 98.0
                self._log.info("VEHICLE", f"[{self._name}] Set Q_ASSIST_SPEED to 30.0 and Q_ASSIST_ALT to 98.0")
            except Exception as pe:
                self._log.warn("VEHICLE", f"[{self._name}] Failed to set QuadPlane assist parameters: {pe}")
            return True
        except Exception as e:
            self._log.error("VEHICLE", f"[{self._name}] Connection failed: {e}")
            return False

    def _supported_mode_names(self) -> set[str]:
        if not self.connected:
            return set()
        try:
            modes = getattr(self._vehicle, "_mode_mapping", None) or {}
            return {str(name).upper() for name in modes.keys()}
        except Exception:
            return set()

    def _wait_for_mode(self, mode_name: str, timeout: float = 15.0) -> bool:
        if not self.connected:
            return False
        deadline = time.time() + timeout
        while time.time() < deadline:
            current = getattr(self._vehicle.mode, "name", str(self._vehicle.mode)).upper()
            if current == mode_name.upper():
                return True
            time.sleep(0.2)
        self._log.warn(
            "VEHICLE",
            f"[{self._name}] Mode timeout: expected={mode_name} current={self._vehicle.mode}",
        )
        return False

    def choose_hover_mode(self) -> str:
        """Araçta desteklenen en uygun VTOL hover modunu seç."""
        modes = self._supported_mode_names()
        for mode_name in ("QLOITER", "QHOVER", "QSTABILIZE", "GUIDED"):
            if not modes or mode_name in modes:
                return mode_name
        return "GUIDED"

    def arm_and_takeoff(self, altitude: float) -> bool:
        """QuadPlane için güvenli VTOL kalkış."""
        if not self.connected:
            return False

        try:
            v = self._vehicle
            hover_mode = self.choose_hover_mode()
            self.set_mode(hover_mode)
            self._log.info("VEHICLE", f"[{self._name}] Arming in {hover_mode}...")

            v.armed = True

            deadline = time.time() + self._arm_timeout
            while not v.armed and time.time() < deadline:
                time.sleep(0.5)

            if not v.armed:
                self._log.error("VEHICLE", f"[{self._name}] Arm timeout!")
                return False

            self.set_mode("GUIDED")
            self._log.info("VEHICLE", f"[{self._name}] Taking off to {altitude}m...")
            msg = v.message_factory.command_long_encode(
                0, 0,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0, 0, 0, 0,
                0, 0, altitude,
            )
            with self._lock:
                v.send_mavlink(msg)
                if hasattr(v, "flush"):
                    v.flush()

            # İrtifa bekleme
            deadline = time.time() + 120.0
            while True:
                alt = v.location.global_relative_frame.alt
                if alt is not None and alt >= altitude * 0.90:
                    self._log.info("VEHICLE",
                        f"[{self._name}] Reached {alt:.1f}m (target: {altitude}m)")
                    break
                if time.time() > deadline:
                    self._log.error("VEHICLE", f"[{self._name}] Takeoff altitude timeout!")
                    return False
                time.sleep(0.5)

            return True
        except Exception as e:
            self._log.error("VEHICLE", f"[{self._name}] Takeoff failed: {e}")
            return False

    def send_velocity(
        self,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
        yaw_rate: float = 0.0,
    ):
        """
        Body frame hız komutu.

        Args:
            vx: ileri/geri (m/s), pozitif ileri
            vy: sol/sağ (m/s), pozitif sağ
            vz: yukarı/aşağı (m/s), pozitif aşağı (NED standardı)
            yaw_rate: yaw hızı (deg/s), saat yönü pozitif
        """
        if not self.connected:
            self._log.warn("VEHICLE", f"[{self._name}] send_velocity FAILED: not connected")
            return

        yaw_rate_rad = math.radians(yaw_rate)

        msg = self._vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000_0100_0111_0000,  # vx, vy, vz, yaw_rate parametreleri
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, yaw_rate_rad,
        )

        with self._lock:
            self._vehicle.send_mavlink(msg)
            if hasattr(self._vehicle, "flush"):
                self._vehicle.flush()
        
        self._log.debug("VEHICLE",
            f"[{self._name}] MAVLink: vx={vx:+.1f} vy={vy:+.1f} vz={vz:+.1f} yaw={yaw_rate:+.1f}°/s",
            debounce=0.1)

    def goto(self, lat: float, lon: float, alt: float):
        """GPS koordinatına git."""
        if not self.connected:
            return

        point = LocationGlobalRelative(lat, lon, alt)
        with self._lock:
            self._vehicle.simple_goto(point)
        self._log.info("VEHICLE",
            f"[{self._name}] Goto lat={lat:.7f} lon={lon:.7f} alt={alt:.1f}")

    def loiter_at(self, lat: float, lon: float, alt: float, radius: float = 100.0) -> bool:
        """GUIDED modunda belirtilen koordinat etrafında daire çizerek (loiter) uç.
        Pozitif radius = saat yönü, negatif = saat yönü tersi."""
        if not self.connected:
            return False

        current_mode = getattr(self._vehicle.mode, "name", str(self._vehicle.mode)).upper()
        if current_mode != "GUIDED":
            self.clear_rc_overrides()
            self.set_mode("GUIDED")

        # Pozitif radius = saat yönü (ArduPilot standardı)
        cw_radius = abs(radius)

        msg = self._vehicle.message_factory.command_long_encode(
            0, 0,
            getattr(mavutil.mavlink, "MAV_CMD_NAV_LOITER_UNLMT", 17),
            0,
            0,            # Param 1 (boş)
            0,            # Param 2 (boş)
            cw_radius,    # Param 3 (Yarıçap, metre - pozitif = saat yönü)
            0,            # Param 4 (Yaw)
            lat,          # Param 5 (Enlem)
            lon,          # Param 6 (Boylam)
            alt,          # Param 7 (İrtifa)
        )
        with self._lock:
            self._vehicle.send_mavlink(msg)
            if hasattr(self._vehicle, "flush"):
                self._vehicle.flush()
        self._log.info(
            "VEHICLE",
            f"[{self._name}] Native loiter commanded: lat={lat:.7f} lon={lon:.7f} alt={alt:.1f}m radius={radius:.1f}m",
            debounce=1.0
        )
        return True

    def goto_with_speed(
        self,
        lat: float,
        lon: float,
        alt: float,
        airspeed: Optional[float] = None,
        groundspeed: Optional[float] = None,
    ):
        """GPS koordinatına belirli hızla git."""
        if not self.connected:
            return

        point = LocationGlobalRelative(lat, lon, alt)
        with self._lock:
            self._vehicle.simple_goto(point, airspeed=airspeed, groundspeed=groundspeed)
        speed_info = ""
        if airspeed is not None:
            speed_info = f" airspeed={airspeed:.1f}"
        elif groundspeed is not None:
            speed_info = f" groundspeed={groundspeed:.1f}"
        self._log.debug(
            "VEHICLE",
            f"[{self._name}] Goto lat={lat:.7f} lon={lon:.7f} alt={alt:.1f}{speed_info}",
            debounce=0.25,
        )

    def set_mode(self, mode: str):
        """Uçuş modunu değiştir."""
        if not self.connected:
            return
        self._vehicle.mode = VehicleMode(mode)
        self._log.info("VEHICLE", f"[{self._name}] Mode → {mode}")
        self._wait_for_mode(mode, timeout=12.0)

    def force_vtol_mode(self):
        """QuadPlane aracını zorla VTOL (Copter) modunda tutar."""
        if not self.connected:
            return
        self._log.info("VEHICLE", f"[{self._name}] Forcing VTOL (MC) mode...")
        msg = self._vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
            0,
            3,  # Multicopter (VTOL) modu
            0, 0, 0, 0, 0, 0
        )
        with self._lock:
            self._vehicle.send_mavlink(msg)

    def transition_to_fw(self):
        """QuadPlane'ı fixed-wing moduna geçir."""
        if not self.connected:
            return
        self._log.info("VEHICLE", f"[{self._name}] Transitioning to FW mode...")
        msg = self._vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
            0,
            4,  # Sabit kanat (FW) modu
            0, 0, 0, 0, 0, 0
        )
        with self._lock:
            self._vehicle.send_mavlink(msg)
            if hasattr(self._vehicle, "flush"):
                self._vehicle.flush()

    def set_throttle_override(self, pwm: int):
        """RC3 override ile arka itkiyi ver."""
        if not self.connected:
            return
        self.set_rc_overrides(throttle_pwm=pwm)

    def set_rc_overrides(
        self,
        roll_pwm: Optional[int] = None,
        pitch_pwm: Optional[int] = None,
        throttle_pwm: Optional[int] = None,
        yaw_pwm: Optional[int] = None,
    ):
        """Fixed-wing için RC override komutu gönder."""
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

        signature = (
            int(current.get("1", 1500)),
            int(current.get("2", 1500)),
            int(current.get("3", 0)),
            int(current.get("4", 1500)),
        )
        if signature != self._last_rc_signature:
            self._log.info(
                "VEHICLE",
                f"[{self._name}] RC override roll={signature[0]} pitch={signature[1]} "
                f"thr={signature[2]} yaw={signature[3]}",
                debounce=0.1,
            )
            self._last_rc_signature = signature

    def clear_rc_overrides(self):
        """RC override değerlerini temizle."""
        if not self.connected:
            return
        self._vehicle.channels.overrides = {}
        self._last_rc_signature = None
        self._log.info("VEHICLE", f"[{self._name}] RC overrides cleared")

    def start_fixed_wing(
        self,
        transition_wait: float = 10.0,
        fw_throttle: Optional[int] = None,
        airspeed: Optional[float] = None,
    ) -> bool:
        """VTOL kalkıştan sonra fixed-wing geçişini güvenli sırayla yap."""
        if not self.connected:
            return False
        try:
            self.set_mode("FBWB")
            self.transition_to_fw()
            if fw_throttle is not None:
                self.set_throttle_override(fw_throttle)
            if airspeed is not None:
                self.set_airspeed(airspeed)
            time.sleep(max(transition_wait, 1.0))
            cruise_throttle = fw_throttle if fw_throttle is not None else 1650
            self.set_rc_overrides(roll_pwm=1500, pitch_pwm=1500, throttle_pwm=cruise_throttle, yaw_pwm=1500)
            self._log.info("VEHICLE", f"[{self._name}] Fixed-wing cruise ready in FBWB")
            return True
        except Exception as e:
            self._log.error("VEHICLE", f"[{self._name}] Fixed-wing start failed: {e}")
            return False

    def set_airspeed(self, airspeed: float):
        """Aracın hedef hava hızını ayarla."""
        if not self.connected:
            return
        try:
            self._vehicle.airspeed = airspeed
            if self._last_airspeed is None or abs(self._last_airspeed - airspeed) >= 0.5:
                self._log.info("VEHICLE", f"[{self._name}] Airspeed → {airspeed:.1f} m/s")
                self._last_airspeed = airspeed
            else:
                self._log.debug("VEHICLE", f"[{self._name}] Airspeed hold {airspeed:.1f} m/s", debounce=0.5)
        except Exception as e:
            self._log.warn("VEHICLE", f"[{self._name}] Airspeed set failed: {e}")

    def is_fixed_wing_mode(self) -> bool:
        """Araç fixed-wing mantığında mı uçuyor."""
        if not self.connected:
            return False
        mode = (self._vehicle.mode.name or "").upper()
        if mode.startswith("Q"):
            return False
        return mode in {"AUTO", "CRUISE", "FBWA", "FBWB", "GUIDED", "LOITER", "RTL"}

    def goto_offset(
        self,
        north_m: float,
        east_m: float,
        alt: float,
        airspeed: Optional[float] = None,
        groundspeed: Optional[float] = None,
    ):
        """Mevcut konumdan N/E offset ile waypoint üret."""
        if not self.connected:
            return
        loc = self._vehicle.location.global_relative_frame
        lat = (loc.lat or 0.0) + north_m / 111111.0
        lon_scale = 111111.0 * math.cos(math.radians(loc.lat or 0.0))
        lon = (loc.lon or 0.0) + east_m / max(lon_scale, 1.0)
        self.goto_with_speed(lat, lon, alt, airspeed=airspeed, groundspeed=groundspeed)

    def fly_fixed_wing_step(
        self,
        forward_speed: float,
        heading_correction_deg: float = 0.0,
        climb_rate: float = 0.0,
        step_time: float = 2.0,
        min_alt: float = 10.0,
        max_alt: float = 55.0,
        max_heading_step_deg: float = 35.0,
    ):
        """
        Fixed-wing araç için kısa vadeli waypoint üret.

        `climb_rate` yukarı pozitif kabul edilir.
        """
        if not self.connected:
            return

        telem = self.get_telemetry()
        heading = telem["heading"] or 0.0
        current_alt = telem["alt"] or min_alt

        heading_delta = float(np.clip(heading_correction_deg, -max_heading_step_deg, max_heading_step_deg))
        target_heading = (heading + heading_delta) % 360.0
        travel_distance = max(forward_speed, 2.0) * max(step_time, 0.5)
        target_alt = float(np.clip(current_alt + climb_rate * step_time, min_alt, max_alt))

        heading_rad = math.radians(target_heading)
        north = travel_distance * math.cos(heading_rad)
        east = travel_distance * math.sin(heading_rad)

        self.goto_offset(
            north_m=north,
            east_m=east,
            alt=target_alt,
            airspeed=max(forward_speed, 2.0),
        )
        self._log.debug(
            "VEHICLE",
            f"[{self._name}] FW step: hdg={heading:.0f}→{target_heading:.0f} "
            f"dist={travel_distance:.1f}m alt={current_alt:.1f}→{target_alt:.1f}m",
            debounce=0.25,
        )

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
        """
        Fixed-wing için seyrek ve kararlı ileri lookahead waypoint komutu üret.

        Aynı hedef çok sık tekrar yazılmaz; böylece uçak gerçekten ileri gider.
        """
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

        self.set_airspeed(airspeed)
        self.goto_offset(north_m=north, east_m=east, alt=target_alt, airspeed=airspeed)
        self._last_fw_command_time = now
        self._last_fw_signature = signature
        self._log.info(
            "VEHICLE",
            f"[{self._name}] FW hold: hdg={target_heading_deg:.0f}° alt={target_alt:.1f}m "
            f"spd={airspeed:.1f} lookahead={travel_distance:.0f}m",
            debounce=0.15,
        )
        return True

    def command_fixed_wing_attitude(
        self,
        target_heading_deg: float,
        target_alt: float,
        cruise_throttle: int = 1650,
        min_throttle: int = 1500,
        max_throttle: int = 1850,
    ) -> bool:
        """
        FBWB modunda fixed-wing'i RC override ile yönlendir.

        Waypoint loiter yerine doğrudan yatış/pitch/throttle verilir.
        """
        if not self.connected:
            return False

        current_mode = getattr(self._vehicle.mode, "name", str(self._vehicle.mode)).upper()
        if current_mode != "FBWB":
            self.set_mode("FBWB")

        telem = self.get_telemetry()
        current_heading = float(telem["heading"] or 0.0)
        current_alt = float(telem["alt"] or target_alt)
        current_speed = float(telem["groundspeed"] or telem["airspeed"] or 0.0)
        heading_error = ((target_heading_deg - current_heading + 540.0) % 360.0) - 180.0
        alt_error = float(target_alt - current_alt)

        roll_pwm = int(1500 + np.tanh(math.radians(heading_error) * 1.3) * 320.0)
        roll_pwm = int(np.clip(roll_pwm, 1180, 1820))

        pitch_pwm = int(1500 + np.tanh(alt_error / 10.0) * 110.0)
        pitch_pwm = int(np.clip(pitch_pwm, 1380, 1620))
        speed_error = max(0.0, 12.0 - current_speed)
        turn_boost = abs(heading_error) * 1.5
        throttle_pwm = int(
            np.clip(
                cruise_throttle + speed_error * 18.0 + turn_boost,
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
            f"[{self._name}] FW attitude: hdg={current_heading:.0f}→{target_heading_deg:.0f} "
            f"err={heading_error:+.0f} alt={current_alt:.1f}→{target_alt:.1f} "
            f"rc=({roll_pwm},{pitch_pwm},{throttle_pwm})",
            debounce=0.1,
        )
        return True

    def send_velocity_with_alt_limit(
        self,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
        yaw_rate: float = 0.0,
        min_alt: float = 10.0,
        max_alt: float = 55.0,
    ):
        """
        Body frame hız komutu - yükseklik limiti ile.
        
        Args:
            vx: ileri/geri (m/s)
            vy: sol/sağ (m/s)
            vz: yukarı/aşağı (m/s), pozitif aşağı
            yaw_rate: yaw hızı (deg/s)
            min_alt: minimum irtifa (m)
            max_alt: maksimum irtifa (m)
        """
        if not self.connected:
            return
            
        # Mevcut irtifa kontrolü
        alt = self._vehicle.location.global_relative_frame.alt or 0
        
        # FW modunda irtifa kaybı kontrolü
        is_fw_mode = hasattr(self._vehicle, 'is_fw_mode') and self._vehicle.is_fw_mode
        
        # Yükseklik limitlerini uygula - FW modunda daha agresif
        if alt < min_alt:
            # Çok alçak - yukarı çık!
            if vz >= 0:  # Aşağı iniyor veya düz - yukarı çıkmaya zorla
                vz = -4.0 if is_fw_mode else -2.0  # Sabit kanatta daha agresif
                self._log.warn("VEHICLE", 
                    f"[{self._name}] LOW ALT {alt:.1f}m < {min_alt}m, forcing climb vz={vz}")
                # FW modunda çok düşükse MC'ye dön
                if is_fw_mode and alt < min_alt - 5:
                    self.force_vtol_mode()
                    self._log.warn("VEHICLE", f"[{self._name}] Emergency: returning to MC mode!")
            # vz < 0 ise (yukarı çıkıyor) izin ver
        elif alt <= min_alt + 2.0 and vz > 0:
            # Minimuma yakın - aşağı inme
            vz = 0
            self._log.warn("VEHICLE", f"[{self._name}] Alt limit {min_alt}m, blocking descent")
        elif alt >= max_alt and vz < 0:  # Çok yüksek, yukarı çıkmayı engelle
            vz = 0
            self._log.warn("VEHICLE", f"[{self._name}] Alt limit {max_alt}m, blocking climb")
        
        self.send_velocity(vx, vy, vz, yaw_rate)

    def get_telemetry(self) -> dict:
        """Mevcut telemetri verilerini al."""
        if not self.connected:
            return {"lat": 0, "lon": 0, "alt": 0, "heading": 0,
                    "airspeed": 0, "groundspeed": 0, "mode": "UNKNOWN",
                    "armed": False}

        loc = self._vehicle.location.global_relative_frame
        return {
            "lat": loc.lat or 0.0,
            "lon": loc.lon or 0.0,
            "alt": loc.alt or 0.0,
            "heading": self._vehicle.heading or 0,
            "airspeed": self._vehicle.airspeed or 0,
            "groundspeed": self._vehicle.groundspeed or 0,
            "mode": self._vehicle.mode.name,
            "armed": self._vehicle.armed,
        }

    def safe_land(self):
        """Güvenli iniş."""
        if not self.connected:
            return
        if self._landing_started:
            return
        self._landing_started = True
        self._log.info("VEHICLE", f"[{self._name}] Landing...")
        try:
            self.force_vtol_mode()
            time.sleep(5.0)
            landing_mode = "QLAND" if "QLAND" in self._supported_mode_names() else "LAND"
            self.set_mode(landing_mode)
        except Exception:
            self._vehicle.mode = VehicleMode("LAND")

    def close(self):
        """Bağlantıyı kapat."""
        if self._vehicle is not None:
            try:
                self._vehicle.close()
            except Exception:
                pass
            self._vehicle = None
            self._connected = False
            self._log.info("VEHICLE", f"[{self._name}] Connection closed")

    @staticmethod
    def distance_between(loc1: dict, loc2: dict) -> float:
        """İki GPS noktası arası mesafe (metre)."""
        import math
        R = 6371000  # Dünya yarıçapı (metre)
        lat1, lon1 = math.radians(loc1["lat"]), math.radians(loc1["lon"])
        lat2, lon2 = math.radians(loc2["lat"]), math.radians(loc2["lon"])
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    @staticmethod
    def bearing_to(loc1: dict, loc2: dict) -> float:
        """loc1'den loc2'ye bearing (derece, 0=kuzey)."""
        import math
        lat1, lon1 = math.radians(loc1["lat"]), math.radians(loc1["lon"])
        lat2, lon2 = math.radians(loc2["lat"]), math.radians(loc2["lon"])
        dlon = lon2 - lon1
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing = math.degrees(math.atan2(x, y))
        return (bearing + 360) % 360
