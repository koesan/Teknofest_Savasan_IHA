#!/usr/bin/env python3
"""
Savaşan İHA - Ana Giriş Noktası

Teknofest 2026 Savaşan İHA Yarışması
Otonom Takip & Kilitlenme Sistemi

Kullanım:
    roslaunch iq_sim multi_drone.launch
    sim_vehicle.py -v ArduPlane -f quadplane --model gazebo-zephyr -I0
    sim_vehicle.py -v ArduPlane -f quadplane --model gazebo-zephyr -I1

    python3 main.py --config config/sim.yaml
    python main.py --no-display
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from datetime import datetime
from typing import Optional

import cv2
import numpy as np

# Proje kök dizini
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, PROJECT_ROOT)

from core.config import Config
from core.logger import init_logger, get_logger
from perception.detector import Detector
from perception.tracker import SingleTargetTracker, TrackState
from perception.lock_evaluator import LockEvaluator, LockStatus
from perception.reacquisition import Reacquisition
from control.visual_servo import VisualServo, ServoCommand
from control.distance_controller import DistanceController
from control.vehicle import Vehicle
from mission.fsm import MissionFSM, MissionState
from mission.search import SearchStrategy
from mission.prey_pilot import PreyPilot
from iol.camera import Camera
from iol.hud import HUD
from iol.server_comms import ServerComms


class HunterSystem:
    """
    Ana avcı İHA sistemi.
    Tüm modülleri birleştirir, ana döngüyü çalıştırır.
    """

    def __init__(self, cfg: Config):
        self.cfg = cfg
        log_path = cfg.general.log_path
        if log_path and not os.path.isabs(log_path):
            log_path = os.path.join(PROJECT_ROOT, log_path)
        self.log = init_logger(
            log_path=log_path,
            level=cfg.general.log_level,
            console=True,
        )
        self.log.info("SYSTEM", "=" * 60)
        self.log.info("SYSTEM", "Savaşan İHA - Otonom Takip & Kilitlenme Sistemi")
        self.log.info("SYSTEM", "=" * 60)

        # --- Zamanlama / yakalama durumu ---
        self._frame_count = 0
        self._fps = 0.0
        self._fps_timer = time.time()
        self._fps_count = 0
        self._lock_sent = False
        self._hunter_fw_mode = False
        self._prey_fw_mode = False
        self._last_search_command_at = 0.0
        self._last_pursuit_command_at = 0.0
        self._last_reacquire_command_at = 0.0
        self._last_fsm_state = MissionState.INIT
        self._capture_enabled = False
        self._capture_dir = ""
        self._capture_seq = 0
        self._last_detect_capture_at = -1e9
        self._last_countdown_capture_at = -1e9
        self._capture_detect_interval = 2.0
        self._capture_countdown_interval = 0.0

        # --- Modüller ---
        self._init_perception(cfg)
        self._init_control(cfg)
        self._init_mission(cfg)
        self._init_io(cfg)

        # --- Araçlar ---
        self.hunter: Optional[Vehicle] = None
        self.prey: Optional[Vehicle] = None

    def _init_perception(self, cfg: Config):
        """Perception modüllerini başlat."""
        self.detector = Detector(
            model_path=cfg.detector.model_path,
            confidence_threshold=cfg.detector.confidence_threshold,
            iou_threshold=cfg.detector.iou_threshold,
            imgsz=cfg.detector.imgsz,
            max_detections=cfg.detector.max_detections,
            target_classes=cfg.detector.target_classes,
        )
        self.tracker = SingleTargetTracker(
            process_noise_pos=cfg.tracker.process_noise_pos,
            process_noise_vel=cfg.tracker.process_noise_vel,
            process_noise_size=cfg.tracker.process_noise_size,
            measurement_noise_pos=cfg.tracker.measurement_noise_pos,
            measurement_noise_size=cfg.tracker.measurement_noise_size,
            gate_threshold=cfg.tracker.gate_threshold,
            min_hits_to_confirm=cfg.tracker.min_hits_to_confirm,
            max_coast_frames=cfg.tracker.max_coast_frames,
            max_coast_time=cfg.tracker.max_coast_time,
            min_bbox_size=cfg.tracker.min_bbox_size,
            frame_width=cfg.camera.frame_width,
            frame_height=cfg.camera.frame_height,
        )
        self.lock_eval = LockEvaluator(
            frame_width=cfg.camera.frame_width,
            frame_height=cfg.camera.frame_height,
            roi_width=cfg.detector.roi_width,   # ROI boyutu için
            roi_height=cfg.detector.roi_height, # ROI boyutu için
            target_area_margin_x=cfg.lock.target_area_margin_x,
            target_area_margin_y=cfg.lock.target_area_margin_y,
            lock_rect_padding=cfg.lock.lock_rect_padding,
            min_target_size_ratio=cfg.lock.min_target_size_ratio,
            min_coverage_ratio=cfg.lock.min_coverage_ratio,
            max_center_offset_ratio=cfg.lock.max_center_offset_ratio,
            required_lock_duration=cfg.lock.required_lock_duration,
            lock_window_duration=cfg.lock.lock_window_duration,
            hold_grace_time=cfg.lock.hold_grace_time,
            bbox_scale_factor=getattr(cfg.lock, 'bbox_scale_factor', 0.0),
        )
        self.reacq = Reacquisition(
            frame_width=cfg.camera.frame_width,
            frame_height=cfg.camera.frame_height,
        )
        self.log.info("SYSTEM", "Perception modules initialized")

    def _init_control(self, cfg: Config):
        """Control modüllerini başlat."""
        c = cfg.control
        self.servo = VisualServo(
            frame_width=cfg.camera.frame_width,
            frame_height=cfg.camera.frame_height,
            yaw_kp=c.yaw_kp, yaw_ki=c.yaw_ki, yaw_kd=c.yaw_kd,
            max_yaw_rate=c.max_yaw_rate,
            yaw_deadband=c.yaw_deadband,
            yaw_integral_limit=c.yaw_integral_limit,
            alt_kp=c.alt_kp, alt_ki=c.alt_ki, alt_kd=c.alt_kd,
            max_vertical_speed=c.max_vertical_speed,
            alt_deadband=c.alt_deadband,
            alt_integral_limit=c.alt_integral_limit,
            max_lateral_speed=c.max_lateral_speed,
            lateral_gain=c.lateral_gain,
            lead_time=c.lead_time,
            fine_error_threshold=c.fine_error_threshold,
            fine_gain_multiplier=c.fine_gain_multiplier,
            max_yaw_delta=c.max_yaw_delta,
            max_speed_delta=c.max_speed_delta,
            max_vz_delta=c.max_vz_delta,
        )
        self.dist_ctrl = DistanceController(
            frame_width=cfg.camera.frame_width,
            frame_height=cfg.camera.frame_height,
            fwd_kp=c.fwd_kp, fwd_ki=c.fwd_ki, fwd_kd=c.fwd_kd,
            max_forward_speed=c.max_forward_speed,
            max_reverse_speed=c.max_reverse_speed,
            desired_width_ratio=c.desired_width_ratio,
            min_width_ratio=c.min_width_ratio,
            max_width_ratio=c.max_width_ratio,
            max_speed_delta=c.max_speed_delta,
        )
        self.log.info("SYSTEM", "Control modules initialized")

    def _init_mission(self, cfg: Config):
        """Mission modüllerini başlat."""
        m = cfg.mission
        self.fsm = MissionFSM(
            detection_confirm_time=m.detection_confirm_time,
            intercept_timeout=m.intercept_timeout,
            reacquire_timeout=m.reacquire_timeout,
            track_loss_grace=m.track_loss_grace,
        )
        self.search = SearchStrategy(
            pattern=m.search_pattern,
            start_leg=m.search_start_leg,
            leg_increment=m.search_leg_increment,
            speed=m.search_speed,
            yaw_scan_rate=m.search_yaw_scan_rate,
            max_radius=m.search_max_radius,
            center_lat=m.patrol_center_lat,
            center_lon=m.patrol_center_lon,
            altitude=m.hunter_altitude,
            yaw_scan_enabled=getattr(m, 'search_yaw_scan_enabled', True),
        )
        self.prey_pilot = PreyPilot(
            center_lat=m.patrol_center_lat,
            center_lon=m.patrol_center_lon,
            patrol_radius=m.prey_patrol_radius,
            altitude=m.prey_altitude,
            min_altitude=getattr(m, 'prey_min_altitude', 30.0),
            step_deg=m.prey_patrol_step_deg,
            period=m.prey_patrol_period,
            fw_throttle=m.prey_fw_throttle,
            hold_altitude=getattr(m, 'prey_hold_altitude', True),
        )
        self.log.info("SYSTEM", "Mission modules initialized")

    def _init_io(self, cfg: Config):
        """IO modüllerini başlat."""
        target_area = self.lock_eval.target_area
        self.hud = HUD(
            frame_width=cfg.camera.frame_width,
            frame_height=cfg.camera.frame_height,
            target_area=target_area,
        )
        self.server = ServerComms()
        self._init_capture(cfg)
        self.log.info("SYSTEM", "IO modules initialized")

    def _init_capture(self, cfg: Config):
        """Hedef tespit ve kilitlenme sayimi icin ekran goruntusu kaydi."""
        capture_dir = str(getattr(cfg.general, "capture_dir", "") or "").strip()
        if capture_dir and not os.path.isabs(capture_dir):
            capture_dir = os.path.join(PROJECT_ROOT, capture_dir)
        self._capture_detect_interval = float(
            getattr(cfg.general, "detection_capture_interval", 2.0)
        )
        self._capture_countdown_interval = float(
            getattr(cfg.general, "countdown_capture_interval", 0.0)
        )

        if not capture_dir:
            self._capture_enabled = False
            return

        self._capture_dir = capture_dir
        try:
            os.makedirs(self._capture_dir, exist_ok=True)
            self._capture_enabled = True
            self.log.info(
                "SYSTEM",
                (
                    "Capture directory ready: "
                    f"{self._capture_dir} "
                    f"(tespit={self._capture_detect_interval:.1f}s "
                    f"sayim={self._capture_countdown_interval:.1f}s)"
                ),
            )
            self._write_capture_probe()
        except OSError as exc:
            self._capture_enabled = False
            self.log.error("SYSTEM", f"Capture directory unavailable: {exc}")

    # =========================================================================
    # Kurulum
    # =========================================================================

    def setup(self) -> bool:
        """Araçları bağla ve kaldır."""
        cfg = self.cfg

        # İHA'lara bağlan
        self.hunter = Vehicle(
            cfg.vehicle.hunter_connection, name="AVCI",
            connection_timeout=cfg.vehicle.connection_timeout,
        )
        self.prey = Vehicle(
            cfg.vehicle.prey_connection, name="AV",
            connection_timeout=cfg.vehicle.connection_timeout,
        )

        if not self.hunter.connect():
            self.log.error("SYSTEM", "Hunter connection failed!")
            return False
        if not self.prey.connect():
            self.log.error("SYSTEM", "Prey connection failed!")
            return False

        # Kameralar için ROS düğümünü başlat
        try:
            import rospy
            rospy.init_node("savasan_iha_system", anonymous=True, disable_signals=True)
            self.log.info("SYSTEM", "ROS node initialized")
        except ImportError:
            self.log.warn("SYSTEM", "rospy not found, falling back to OpenCV only")
        except Exception as e:
            self.log.warn("SYSTEM", f"ROS init warning: {e}")

        # Kameralar
        self.hunter_cam = Camera(
            cfg.camera.hunter_topic, name="hunter_front",
            width=cfg.camera.frame_width, height=cfg.camera.frame_height,
        )

        # Kalkış
        self.fsm.trigger_takeoff()
        self.log.info("SYSTEM", "Starting takeoff sequence...")

        if not self.prey.arm_and_takeoff(cfg.mission.prey_altitude):
            self.log.error("SYSTEM", "Prey takeoff failed!")
            return False
        prey_fw_speed = max(cfg.mission.prey_fw_throttle / 160.0, 10.0)
        if not self.prey.start_fixed_wing(
            transition_wait=cfg.mission.prey_transition_wait,
            fw_throttle=cfg.mission.prey_fw_throttle,
            airspeed=prey_fw_speed,
        ):
            self.log.error("SYSTEM", "Prey fixed-wing transition failed!")
            return False
        self._prey_fw_mode = True

        if not self.hunter.arm_and_takeoff(cfg.mission.hunter_altitude):
            self.log.error("SYSTEM", "Hunter takeoff failed!")
            return False

        hunter_transition_wait = getattr(cfg.mission, "hunter_transition_wait", cfg.mission.prey_transition_wait)
        hunter_fw_throttle = getattr(cfg.mission, "hunter_fw_throttle", 1700)
        if not self.hunter.start_fixed_wing(
            transition_wait=hunter_transition_wait,
            fw_throttle=hunter_fw_throttle,
            airspeed=max(cfg.mission.search_speed, 12.0),
        ):
            self.log.error("SYSTEM", "Hunter fixed-wing transition failed!")
            return False
        self._hunter_fw_mode = True

        self.fsm.trigger_takeoff_complete()
        self.search.start()
        self.log.info("SYSTEM", "Setup complete, entering main loop")
        return True

    # =========================================================================
    # Ana Döngü
    # =========================================================================

    def run(self):
        """Ana döngü."""
        if not self.setup():
            self.log.error("SYSTEM", "Setup failed, aborting")
            return

        cfg = self.cfg
        start_time = time.time()

        try:
            while True:
                now = time.time()
                self._update_fps(now)

                # --- 1. Av devriyesi ---
                self.prey_pilot.update(self.prey, now)

                # --- 2. Görüntü yakalama ---
                frame = self.hunter_cam.read()
                if frame is None:
                    self._handle_no_frame(now)
                    time.sleep(cfg.general.loop_sleep)
                    continue

                # --- 3. Tespit et ---
                roi = None
                if self.tracker.is_active:
                    roi = self.tracker.get_predicted_roi(
                        dt_ahead=0.10,
                        expand=getattr(cfg.detector, "roi_expand_scale", 1.8),
                    )
                elif self.fsm.state == MissionState.REACQUIRE:
                    roi = self.reacq.get_search_roi()

                detections = self.detector.detect_with_roi_fallback(
                    frame, predicted_roi=roi,
                    roi_imgsz=cfg.detector.roi_imgsz,
                )

                # En iyi tespit
                best_det = detections[0] if detections else None

                # --- 4. Takip et ---
                track_result = self.tracker.update(detections, now)
                has_detection = best_det is not None
                track_active = track_result.state in (TrackState.CONFIRMED, TrackState.COASTING)
                effective_bbox = self._select_effective_bbox(track_result, best_det)

                # --- 5. Kilitlenmeyi değerlendir ---
                lock_status = self.lock_eval.evaluate(
                    effective_bbox if effective_bbox is not None else None,
                    now,
                )

                # --- 6. FSM (Durum Makinesi) güncelle ---
                prev_state = self.fsm.state
                self.fsm.update(
                    track_state=track_result.state,
                    has_detection=has_detection,
                    lock_progress=lock_status.lock_progress,
                    is_locked=lock_status.is_locked,
                    now=now,
                )
                if self.fsm.state != prev_state:
                    self._handle_state_transition(prev_state, self.fsm.state)

                # --- 7. Kontrol et ---
                self._execute_control(track_result, lock_status, now, effective_bbox)

                # --- 8. Yeniden yakalama yönetimi ---
                if track_result.state == TrackState.LOST:
                    # Takipçi KAYIP (LOST) olduğunda lock_evaluator'ı da sıfırla
                    # Böylece hedef kaybolduğunda bbox ve lock durumu hemen temizlenir
                    if self.lock_eval._is_locked or self.lock_eval._lock_start is not None:
                        self.log.info("TRACK", "Target LOST - resetting lock evaluator")
                        self.lock_eval.reset()
                    
                    if self.fsm.state == MissionState.REACQUIRE:
                        if not self.reacq._lost_time:
                            last_valid = self.tracker.get_last_valid_result()
                            reacq_source = last_valid if last_valid is not None else track_result
                            reacq_bbox = reacq_source.bbox
                            self.reacq.start(
                                center=reacq_source.centroid,
                                velocity=reacq_source.velocity,
                                size=(
                                    max(float(reacq_bbox[2] - reacq_bbox[0]), 30.0),
                                    max(float(reacq_bbox[3] - reacq_bbox[1]), 30.0),
                                ),
                            )
                elif track_result.state == TrackState.CONFIRMED:
                    self.reacq.reset()

                # --- 9. Sunucu haberleşmesi ---
                self._send_telemetry(now, lock_status, effective_bbox)
                if lock_status.just_locked:
                    self.server.send_lock_packet(
                        target_id="UAV_001",
                        lock_rect=lock_status.lock_rect or (0, 0, 0, 0),
                        lock_duration=lock_status.lock_duration,
                        now=now,
                    )
                    self._lock_sent = True
                elif not lock_status.is_locked:
                    self._lock_sent = False

                # --- 10. Görselleştirme / Kayıt ---
                hunter_telem = self.hunter.get_telemetry() if self.hunter else {}
                mission_elapsed = now - start_time if start_time else 0.0
                mission_failed = self.fsm.state == MissionState.LAND and not lock_status.is_locked
                display_bbox = self._display_bbox(track_result, effective_bbox)
                raw_bbox_display = np.array(best_det.bbox, dtype=float) if best_det is not None else None
                display_frame = None

                if cfg.general.display or self._capture_enabled:
                    display_frame = self.hud.draw(
                        frame=frame,
                        mission_state=self.fsm.state,
                        lock_status=lock_status,
                        track_bbox=display_bbox,
                        raw_bbox=raw_bbox_display,
                        track_confidence=track_result.confidence,
                        server_time=self.server.server_time,
                        fps=self._fps,
                        mission_elapsed=mission_elapsed,
                        mission_failed=mission_failed,
                        vehicle_mode=hunter_telem.get("mode", ""),
                        altitude_m=float(hunter_telem.get("alt", 0.0) or 0.0),
                        airspeed_mps=float(hunter_telem.get("airspeed", 0.0) or 0.0),
                        groundspeed_mps=float(hunter_telem.get("groundspeed", 0.0) or 0.0),
                    )

                if display_frame is not None:
                    self._handle_capture(
                        frame=display_frame,
                        now=now,
                        lock_status=lock_status,
                        has_detection=has_detection,
                        track_active=track_active,
                        effective_bbox=effective_bbox,
                    )

                if cfg.general.display and display_frame is not None:
                    cv2.imshow("Savasan IHA - Hunter", display_frame)
                    cv2.setWindowTitle("Savasan IHA - Hunter", 
                        f"Savasan IHA - {self.fsm.state.name} | FPS: {self._fps:.0f}")
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break

                # --- 11. Günlük Kaydı ---
                self._log_loop(now, track_result, lock_status, start_time, effective_bbox)

                # --- 12. Görev tamamlanma kontrolü ---
                if lock_status.is_locked:
                    self.log.info("MISSION",
                        f"KILITLENME BASARILI! duration={lock_status.lock_duration:.1f}s")
                    # Takibe devam et - durma, birden fazla kilitlenmeye izin ver

                self._frame_count += 1
                time.sleep(cfg.general.loop_sleep)

        except KeyboardInterrupt:
            self.log.info("SYSTEM", "Interrupted by user")
        finally:
            self._cleanup()

    # =========================================================================
    # Kontrol yürütme
    # =========================================================================

    def _execute_control(self, track_result, lock_status: LockStatus, now: float, effective_bbox=None):
        """Duruma göre kontrol komutu ver."""
        state = self.fsm.state

        if state == MissionState.SEARCH:
            self._do_search(now)

        elif state in (MissionState.DETECTED, MissionState.INTERCEPT):
            self._do_intercept(track_result, now, effective_bbox)

        elif state in (MissionState.TRACK, MissionState.LOCK_HOLD):
            self._do_track(track_result, lock_status, now, effective_bbox)

        elif state == MissionState.LOCKED:
            self._do_track(track_result, lock_status, now, effective_bbox)

        elif state == MissionState.REACQUIRE:
            self._do_reacquire(track_result, now)

        elif state == MissionState.LAND:
            self.hunter.safe_land()

    def _do_search(self, now: float):
        """Arama modunda alanı fixed-wing waypoint'lerle tara."""
        if now - self._last_search_command_at < getattr(self.cfg.mission, "search_command_period", 2.0):
            return

        hunter_telem = self.hunter.get_telemetry()
        wp = self.search.get_next_search_waypoint(
            hunter_lat=hunter_telem["lat"],
            hunter_lon=hunter_telem["lon"],
            now=now,
            reach_radius_m=getattr(self.cfg.mission, "search_wp_reached_radius", 18.0),
        )
        target_bearing = Vehicle.bearing_to(
            hunter_telem,
            {"lat": wp[0], "lon": wp[1]},
        )
        self.hunter.command_fixed_wing_attitude(
            target_heading_deg=target_bearing,
            target_alt=wp[2],
            cruise_throttle=getattr(self.cfg.mission, "hunter_fw_throttle", 1700),
        )
        self._last_search_command_at = now
        self.log.info(
            "CONTROL",
            f"SEARCH_CMD: lat={wp[0]:.7f} lon={wp[1]:.7f} alt={wp[2]:.1f} "
            f"hdg={target_bearing:.0f} thr={getattr(self.cfg.mission, 'hunter_fw_throttle', 1700)}",
            debounce=0.3,
        )

    def _do_intercept(self, track_result, now: float, effective_bbox=None):
        """Hedef tespit edildi, yanaş - yükseklik limitli."""
        if track_result.state in (TrackState.CONFIRMED, TrackState.COASTING):
            target_cx, target_cy = self._target_center(track_result, effective_bbox)
            bbox = effective_bbox if effective_bbox is not None else track_result.bbox
            w = max(bbox[2] - bbox[0], 1.0) if bbox is not None else 0.0

            cmd = self.servo.compute(
                target_cx=target_cx,
                target_cy=target_cy,
                target_vx=track_result.velocity[0],
                target_vy=track_result.velocity[1],
                bbox_width=w,
                desired_width_ratio=self.cfg.control.desired_width_ratio,
                now=now,
            )
            
            fwd = self.dist_ctrl.compute(
                bbox_width=w,
                x_error=cmd.x_error,
                y_error=cmd.y_error,
            )
            
            width_ratio = w / float(self.cfg.camera.frame_width)
            if width_ratio < max(self.cfg.control.min_width_ratio * 0.8, 0.012):
                fwd = max(fwd, 9.0 if abs(cmd.x_error) < 0.35 else 7.0)
            
            m = self.cfg.mission
            self._send_hunter_fw_command(
                forward_speed=fwd,
                yaw_rate=cmd.yaw_rate,
                climb_rate=-cmd.vz,
                lateral_speed=cmd.lateral_speed,
                x_error=cmd.x_error,
                y_error=cmd.y_error,
                bbox_width=w,
                min_alt=getattr(m, 'hunter_min_altitude', 10.0),
                max_alt=m.max_hunter_altitude,
            )
            
            self.log.debug("CONTROL",
                f"INTERCEPT cmd: yaw={cmd.yaw_rate:+.1f} vz={cmd.vz:+.1f} "
                f"fwd={fwd:+.1f} lat={cmd.lateral_speed:+.1f} "
                f"w={w:.0f}px err=({cmd.x_error:+.2f},{cmd.y_error:+.2f})",
                debounce=0.2)

    def _do_track(self, track_result, lock_status: LockStatus, now: float, effective_bbox=None):
        """Aktif takip ve lock hold - yükseklik limitli."""
        m = self.cfg.mission
        min_alt = getattr(m, 'hunter_min_altitude', 10.0)
        
        if track_result.state in (TrackState.CONFIRMED, TrackState.COASTING):
            target_cx, target_cy = self._target_center(track_result, effective_bbox)
            bbox = effective_bbox if effective_bbox is not None else track_result.bbox
            w_raw = max(bbox[2] - bbox[0], 1.0) if bbox is not None else 0.0
            
            # Lock Evaluator ile tam senkronizasyon için genişletilmiş genişliği kullan
            scale_mult = 1.0 + getattr(self.lock_eval, "_bbox_scale_factor", 0.0)
            w = w_raw * scale_mult

            cmd = self.servo.compute(
                target_cx=target_cx,
                target_cy=target_cy,
                target_vx=track_result.velocity[0],
                target_vy=track_result.velocity[1],
                bbox_width=w,
                desired_width_ratio=self.cfg.control.desired_width_ratio,
                now=now,
            )

            fwd = self.dist_ctrl.compute(
                bbox_width=w,
                x_error=cmd.x_error,
                y_error=cmd.y_error,
            )
            
            self._send_hunter_fw_command(
                forward_speed=fwd,
                yaw_rate=cmd.yaw_rate,
                climb_rate=-cmd.vz,
                lateral_speed=cmd.lateral_speed,
                x_error=cmd.x_error,
                y_error=cmd.y_error,
                bbox_width=w,
                min_alt=min_alt,
                max_alt=m.max_hunter_altitude,
            )

            self.log.debug("CONTROL",
                f"TRACK cmd: yaw={cmd.yaw_rate:+.1f} vz={cmd.vz:+.1f} "
                f"fwd={fwd:+.1f} lat={cmd.lateral_speed:+.1f} "
                f"err=({cmd.x_error:+.2f},{cmd.y_error:+.2f})",
                debounce=0.2)
        else:
            self._do_search(now)

    def _do_reacquire(self, track_result, now: float):
        """Hedef kayıp - aktif arama, yükseklik limitli."""
        m = self.cfg.mission
        min_alt = getattr(m, 'hunter_min_altitude', 10.0)
        
        if now - self._last_reacquire_command_at < getattr(self.cfg.mission, "reacquire_command_period", 1.2):
            return

        # Son bilinen pozisyona doğru hareket et
        if self.reacq._last_known_center is not None:
            # Tahmini pozisyona git
            cx, cy = self.reacq._last_known_center
            vx, vy = self.reacq._last_known_velocity or (0, 0)
            elapsed = now - self.reacq._lost_time if self.reacq._lost_time else 0
            
            # Hız tahminiyle pozisyon
            pred_cx = cx + vx * elapsed
            pred_cy = cy + vy * elapsed
            
            x_err = (pred_cx - 640) / 640  # normalized [-1, 1]
            climb_rate = -((pred_cy - 360) / 360) * 2.5
            yaw_rate = x_err * 35.0

            self._send_hunter_fw_command(
                forward_speed=6.0,
                yaw_rate=yaw_rate,
                climb_rate=climb_rate,
                lateral_speed=0.0,
                x_error=x_err,
                y_error=(pred_cy - 360) / 360,
                bbox_width=0.0,
                min_alt=min_alt,
                max_alt=m.max_hunter_altitude,
            )
            self.log.debug("REACQ",
                f"Moving to predicted pos=({pred_cx:.0f},{pred_cy:.0f}) yaw={yaw_rate:+.1f}°/s")
        else:
            hunter_telem = self.hunter.get_telemetry()
            target_heading = (hunter_telem["heading"] + getattr(self.cfg.mission, "reacquire_turn_deg", 25.0)) % 360.0
            self.hunter.command_fixed_wing_attitude(
                target_heading_deg=target_heading,
                target_alt=max(min_alt, self.cfg.mission.hunter_altitude),
                cruise_throttle=getattr(self.cfg.mission, "hunter_fw_throttle", 1700),
            )
        self._last_reacquire_command_at = now

    def _send_hunter_fw_command(
        self,
        forward_speed: float,
        yaw_rate: float,
        climb_rate: float,
        lateral_speed: float,
        x_error: float,
        y_error: float,
        bbox_width: float,
        min_alt: float,
        max_alt: float,
    ):
        """Görüntü tabanlı komutu fixed-wing waypoint komutuna çevir."""
        command_period = getattr(self.cfg.mission, "pursuit_command_period", 0.06)
        if time.time() - self._last_pursuit_command_at < command_period:
            return

        width_ratio = bbox_width / float(self.cfg.camera.frame_width)
        edge_error = max(abs(x_error), abs(y_error))
        
        # yaw_rate, geometrik yaw/heading değişimini doğrudan temsil eder.
        heading_correction = yaw_rate

        # Hedef küçükse yatay yönelimi sabitlemek için dikey hareketi sınırla.
        if width_ratio < 0.040:
            climb_rate *= 0.6

        # Hedef mesafesine göre hız profili
        desired_ratio = getattr(self.cfg.control, 'desired_width_ratio', 0.055)
        
        if width_ratio < 0.020:
            # Çok uzak — tam gaz yaklaş
            commanded_speed = max(forward_speed, 14.0)
        elif width_ratio < 0.035:
            # Uzak — hızlı yaklaş
            commanded_speed = max(forward_speed, 11.0)
        elif width_ratio < desired_ratio:
            # Orta mesafe — normal yaklaş
            commanded_speed = max(forward_speed, 8.0)
        elif width_ratio < desired_ratio * 1.3:
            # Hedef mesafede — av hızına ayarla (mesafe koru)
            commanded_speed = min(max(forward_speed, 6.0), 8.0)
        else:
            # Çok yakın — yavaşla, avı geçme
            commanded_speed = min(forward_speed, 5.0)

        # Hedef köşeye kaçıyorsa yavaşla
        if edge_error > 0.85:
            commanded_speed = min(commanded_speed, 7.0)
        elif edge_error > 0.65:
            commanded_speed = min(commanded_speed, 9.0)

        hunter_telem = self.hunter.get_telemetry()
        current_heading = float(hunter_telem["heading"] or 0.0)
        current_alt = float(hunter_telem["alt"] or self.cfg.mission.hunter_altitude)
        target_heading = (current_heading + heading_correction) % 360.0
        
        climb_limit = 2.4 if edge_error > 0.45 else 1.8
        target_alt = float(np.clip(current_alt + np.clip(climb_rate, -climb_limit, climb_limit) * 1.6, min_alt, max_alt))
        sent = self.hunter.command_fixed_wing_attitude(
            target_heading_deg=target_heading,
            target_alt=target_alt,
            cruise_throttle=int(
                np.clip(
                    getattr(self.cfg.mission, "hunter_fw_throttle", 1700) + (commanded_speed - 11.0) * 28.0,
                    1500 if width_ratio > desired_ratio else 1575,
                    1850,
                )
            ),
        )
        self._last_pursuit_command_at = time.time()
        self.log.debug(
            "CONTROL",
            f"PURSUIT: x_err={x_error:+.2f} y_err={y_error:+.2f} width_ratio={width_ratio:.4f} "
            f"turn={heading_correction:+.1f} tgt_hdg={target_heading:.1f} "
            f"tgt_alt={target_alt:.1f} speed={commanded_speed:.1f} sent={sent}",
            debounce=0.2,
        )

    def _handle_no_frame(self, now: float):
        """Kamera verisi yok."""
        self.log.warn("CAMERA", "No frame received", debounce=1.0)
        if self.fsm.state == MissionState.SEARCH:
            self._do_search(now)

    def _handle_capture(
        self,
        frame: np.ndarray,
        now: float,
        lock_status: LockStatus,
        has_detection: bool,
        track_active: bool,
        effective_bbox,
    ):
        """Tespit ve sayim icin goruntu kaydet."""
        if not self._capture_enabled:
            return

        countdown_active = lock_status.all_criteria_met or lock_status.lock_duration > 0.0
        detection_active = has_detection or track_active or effective_bbox is not None

        if lock_status.just_locked:
            self._last_countdown_capture_at = now
            self.log.info("CAPTURE", "Trigger: kilitlenme", debounce=0.01)
            self._save_capture(frame, now, "kilitlenme")
            return

        if countdown_active:
            elapsed = now - self._last_countdown_capture_at
            if self._capture_countdown_interval <= 0.0 or elapsed >= self._capture_countdown_interval:
                self._last_countdown_capture_at = now
                self.log.debug(
                    "CAPTURE",
                    f"Trigger: sayim lock={lock_status.lock_duration:.2f}s elapsed={elapsed:.2f}s",
                    debounce=0.05,
                )
                self._save_capture(frame, now, "sayim")

        if (not countdown_active) and detection_active and now - self._last_detect_capture_at >= self._capture_detect_interval:
            self._last_detect_capture_at = now
            self.log.info(
                "CAPTURE",
                (
                    f"Trigger: tespit lock={lock_status.lock_duration:.2f}s "
                    f"det={int(has_detection)} track={int(track_active)} "
                    f"bbox={int(effective_bbox is not None)}"
                ),
                debounce=0.1,
            )
            self._save_capture(frame, now, "tespit")

    def _save_capture(self, frame: np.ndarray, now: float, reason: str):
        """HUD goruntusunu disk uzerine kaydet."""
        try:
            os.makedirs(self._capture_dir, exist_ok=True)
        except OSError as exc:
            self.log.error("CAPTURE", f"Capture directory error: {exc}")
            return

        timestamp = datetime.fromtimestamp(now).strftime("%Y%m%d_%H%M%S_%f")[:-3]
        filename = f"{timestamp}_{reason}_{self._capture_seq:05d}.png"
        path = os.path.join(self._capture_dir, filename)
        self._capture_seq += 1

        frame_to_save = np.ascontiguousarray(frame)
        ok = cv2.imwrite(path, frame_to_save)
        if not ok:
            encoded_ok, buffer = cv2.imencode(".png", frame_to_save)
            if encoded_ok:
                try:
                    with open(path, "wb") as f:
                        f.write(buffer.tobytes())
                    ok = True
                except OSError as exc:
                    self.log.error("CAPTURE", f"Failed to write encoded capture: {exc}")
        if ok:
            self.log.info(
                "CAPTURE",
                f"Saved {reason}: {path}",
                debounce=0.15 if reason == "sayim" else 1.0,
            )
        else:
            self.log.error("CAPTURE", f"Failed to save capture: {path}")

    def _write_capture_probe(self):
        """Baslangicta yazma yolunu dogrulamak icin test PNG'si uret."""
        probe = np.zeros((64, 192, 3), dtype=np.uint8)
        cv2.putText(
            probe,
            "capture-probe",
            (8, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        self._save_capture(probe, time.time(), "probe")

    def _display_bbox(self, track_result, effective_bbox):
        """HUD icin ham hedef kutusunu dondur."""
        if effective_bbox is not None:
            return np.array(effective_bbox, dtype=float)
        if track_result.state in (TrackState.CONFIRMED, TrackState.COASTING):
            return np.array(track_result.bbox, dtype=float)
        return None

    # =========================================================================
    # Telemetri ve Günlük Kaydı
    # =========================================================================

    def _send_telemetry(self, now: float, lock_status: LockStatus, effective_bbox=None):
        """Sunucuya telemetri gönder."""
        if self.hunter and self.hunter.connected:
            telem = self.hunter.get_telemetry()
            self.server.send_telemetry(
                lat=telem["lat"], lon=telem["lon"],
                alt=telem["alt"], heading=telem["heading"],
                mode=telem["mode"],
                is_autonomous=1 if self.fsm.state != MissionState.LAND else 0,
                is_locking=1 if lock_status.all_criteria_met or lock_status.lock_duration > 0.0 else 0,
                target_bbox=effective_bbox,
                now=now,
            )

    def _log_loop(self, now: float, track_result, lock_status: LockStatus, start_time: float, effective_bbox=None):
        """Periyodik loop log - detaylı bilgiler."""
        # Temel bilgiler
        state = self.fsm.state.name
        track_state = track_result.state.name
        
        # Tracker detayları
        track_info = f"track={track_state} conf={track_result.confidence:.2f}"
        bbox_for_log = effective_bbox if effective_bbox is not None else track_result.bbox
        if bbox_for_log is not None:
            w = bbox_for_log[2] - bbox_for_log[0]
            h = bbox_for_log[3] - bbox_for_log[1]
            track_info += f" size=({w:.0f}x{h:.0f})"
        track_info += (
            f" vel=({track_result.velocity[0]:.0f},{track_result.velocity[1]:.0f})"
            f" hits={track_result.hits} coast={track_result.coast_time:.1f}s"
        )
        
        # Lock durumu
        lock_info = (
            f"lock={lock_status.lock_duration:.1f}s/{lock_status.lock_progress:.0%}"
            f" boyut=%{lock_status.target_size_ratio*100:.2f}"
            f" kapsama=%{lock_status.coverage_ratio*100:.1f}"
            f" sapma=({lock_status.center_offset_px[0]:.0f},{lock_status.center_offset_px[1]:.0f})px"
        )
        if lock_status.all_criteria_met:
            lock_info += " [CRITERIA_OK]"
        if lock_status.is_locked:
            lock_info += " [LOCKED!]"
        
        # Hunter telemetri
        hunter_telem = self.hunter.get_telemetry() if self.hunter else {}
        pos_info = (
            f"alt={hunter_telem.get('alt', 0):.0f}m "
            f"hdg={hunter_telem.get('heading', 0):.0f}° "
            f"spd={hunter_telem.get('airspeed', 0):.1f}/{hunter_telem.get('groundspeed', 0):.1f} "
            f"mode={hunter_telem.get('mode', 'UNK')}"
        )
        
        # Hedef mesafesi tahmini (bbox genişliğinden, lineer ters orantı)
        dist_info = ""
        if bbox_for_log is not None:
            w = bbox_for_log[2] - bbox_for_log[0]
            h = bbox_for_log[3] - bbox_for_log[1]
            width_ratio = max(w, 1.0) / 1280.0
            # Lineer ters orantı: küçük bbox = uzak hedef
            if width_ratio > 1e-5:
                est_dist = 2.8 / width_ratio  # ~70m'de width_ratio≈0.04
                dist_info = f"est_dist~{est_dist:.0f}m"

        prey_info = ""
        if self.prey and self.prey.connected and self.hunter and self.hunter.connected:
            prey_telem = self.prey.get_telemetry()
            if prey_telem["lat"] and hunter_telem.get("lat"):
                actual_dist = Vehicle.distance_between(hunter_telem, prey_telem)
                bearing = Vehicle.bearing_to(hunter_telem, prey_telem)
                heading_err = ((bearing - hunter_telem.get("heading", 0) + 540.0) % 360.0) - 180.0
                prey_info = (
                    f" prey_dist={actual_dist:.0f}m prey_brg={bearing:.0f}°"
                    f" prey_rel={heading_err:+.0f}° prey_alt={prey_telem.get('alt', 0):.0f}m"
                )

        center_cx, center_cy = self._target_center(track_result, bbox_for_log)
        self.log.info("LOOP",
            f"[{state}] {track_info} "
            f"center=({center_cx:.0f},{center_cy:.0f}) "
            f"{lock_info} {pos_info} {dist_info}{prey_info} "
            f"fps={self._fps:.0f} t={now - start_time:.1f}s",
            debounce=0.3)

    def _select_effective_bbox(self, track_result, best_det):
        """Lock ve mesafe hesabında track bbox yerine gerektiğinde güncel detection bbox'ını kullan."""
        track_bbox = None
        if track_result.state in (TrackState.CONFIRMED, TrackState.COASTING):
            track_bbox = track_result.bbox

        det_bbox = None
        if best_det is not None and getattr(best_det, "bbox", None) is not None:
            det_bbox = np.array(best_det.bbox, dtype=float)

        if det_bbox is None:
            return track_bbox
        if track_bbox is None:
            return det_bbox

        track_w = max(float(track_bbox[2] - track_bbox[0]), 1.0)
        track_h = max(float(track_bbox[3] - track_bbox[1]), 1.0)
        det_w = max(float(det_bbox[2] - det_bbox[0]), 1.0)
        det_h = max(float(det_bbox[3] - det_bbox[1]), 1.0)
        track_area = track_w * track_h
        det_area = det_w * det_h
        track_cx = float((track_bbox[0] + track_bbox[2]) / 2.0)
        track_cy = float((track_bbox[1] + track_bbox[3]) / 2.0)
        det_cx = float((det_bbox[0] + det_bbox[2]) / 2.0)
        det_cy = float((det_bbox[1] + det_bbox[3]) / 2.0)
        center_jump = float(np.hypot(det_cx - track_cx, det_cy - track_cy))

        min_effective_w = max(12.0, track_w * 0.55)
        min_effective_h = max(10.0, track_h * 0.55)
        max_center_jump = max(80.0, max(track_w, track_h) * 4.0)

        if det_w < min_effective_w or det_h < min_effective_h:
            self.log.debug(
                "TRACK",
                f"EFFECTIVE KEEP: reject tiny det=({det_w:.0f}x{det_h:.0f}) "
                f"track=({track_w:.0f}x{track_h:.0f})",
                debounce=0.2,
            )
            return track_bbox

        if center_jump > max_center_jump and det_area < track_area * 1.6:
            self.log.debug(
                "TRACK",
                f"EFFECTIVE KEEP: reject jump det=({det_w:.0f}x{det_h:.0f}) "
                f"jump={center_jump:.0f}px track=({track_w:.0f}x{track_h:.0f})",
                debounce=0.2,
            )
            return track_bbox

        if det_area >= track_area * 1.35 or (track_result.state == TrackState.COASTING and det_area >= track_area * 0.85):
            self.log.debug(
                "TRACK",
                f"EFFECTIVE BBOX: det=({det_w:.0f}x{det_h:.0f}) track=({track_w:.0f}x{track_h:.0f}) "
                f"state={track_result.state.name}",
                debounce=0.2,
            )
            return det_bbox

        return track_bbox

    @staticmethod
    def _target_center(track_result, bbox):
        """Servo ve log için kullanılacak hedef merkezi."""
        if bbox is None:
            return float(track_result.centroid[0]), float(track_result.centroid[1])
        cx = float((bbox[0] + bbox[2]) / 2.0)
        cy = float((bbox[1] + bbox[3]) / 2.0)
        return cx, cy

    def _handle_state_transition(self, old_state: MissionState, new_state: MissionState):
        """Kilitlenme sonrasi temiz gecisler."""
        self._last_fsm_state = new_state

        if old_state == MissionState.LOCKED and new_state == MissionState.SEARCH:
            self.log.info("FSM", "Yeni hedef aramasi icin takip ve kilit durumu sifirlaniyor")
            self.tracker.reset()
            self.lock_eval.reset()
            self.reacq.reset()
            self.servo.reset()
            self.dist_ctrl.reset()
            self._lock_sent = False

    def _update_fps(self, now: float):
        """FPS hesapla."""
        self._fps_count += 1
        elapsed = now - self._fps_timer
        if elapsed >= 1.0:
            self._fps = self._fps_count / elapsed
            self._fps_count = 0
            self._fps_timer = now

    # =========================================================================
    # Temizleme
    # =========================================================================

    def _cleanup(self):
        """Kaynakları temizle."""
        self.log.info("SYSTEM", "Shutting down...")
        cv2.destroyAllWindows()

        if self.cfg.general.land_on_exit:
            self.log.info("SYSTEM", "Landing vehicles...")
            if self.hunter:
                self.hunter.safe_land()
            if self.prey:
                self.prey.safe_land()
            time.sleep(3)

        if self.hunter:
            self.hunter.close()
        if self.prey:
            self.prey.close()
        if hasattr(self, 'hunter_cam'):
            self.hunter_cam.close()

        self.log.info("SYSTEM", "Shutdown complete")
        self.log.close()


# =============================================================================
# Komut satırı arayüzü (CLI)
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Savaşan İHA - Otonom Takip & Kilitlenme Sistemi"
    )
    parser.add_argument(
        "--config",
        default=os.path.join(PROJECT_ROOT, "config", "default.yaml"),
        help="Ana konfigürasyon dosyası",
    )
    parser.add_argument(
        "--overlay",
        default=None,
        help="Override konfigürasyon dosyası (default üzerine yazar)",
    )
    parser.add_argument(
        "--no-display",
        action="store_true",
        help="GUI gösterme",
    )
    args = parser.parse_args()

    # Konfigürasyonu yükle
    config_path = args.config
    if not os.path.isabs(config_path):
        config_path = os.path.join(PROJECT_ROOT, config_path)

    overlay_path = args.overlay
    if overlay_path and not os.path.isabs(overlay_path):
        overlay_path = os.path.join(PROJECT_ROOT, overlay_path)

    cfg = Config.from_yaml(config_path, overlay_path=overlay_path)

    # Komut satırı ezmeleri
    if args.no_display:
        cfg._data["general"]["display"] = False

    # Çalıştır
    system = HunterSystem(cfg)
    system.run()


if __name__ == "__main__":
    main()
