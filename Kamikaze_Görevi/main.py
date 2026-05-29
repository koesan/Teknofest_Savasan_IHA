#!/usr/bin/env python3
"""QuadPlane VTOL için Kamikaze Dalış Görevi Kontrolcüsü.

Uçuş Sıralaması:
  TAKEOFF → CRUISE_FORWARD (K'ye basınca) →
  TURN_TO_TARGET (Hedefe yönel) →
  FW_APPROACH (Hedefe düz yaklaşım) →
  FW_DIVE (Dalış) →
  PULLUP (Tırmanma) → CRUISE_FORWARD
"""
from __future__ import annotations

import math
import os
import queue
import sys
import threading
import time
from typing import Optional

import cv2
import numpy as np
import yaml

from camera import Camera
from guidance import DiveGuidance
from gui import GUI
from logger import init_logger
from qr_detector import QRDetector, QRResult
from vehicle import Vehicle


class SimpleQRWorker:
    """Arka planda sadece en son gönderilen kamera çerçevesini işler."""

    def __init__(self, detector: QRDetector):
        self._detector = detector
        self._input_q: queue.Queue[Optional[np.ndarray]] = queue.Queue(maxsize=1)
        self._lock = threading.Lock()
        self._latest = QRResult()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._submitted = 0
        self._dropped = 0
        self._processed = 0
        self._seen = 0
        self._decoded = 0
        self._errors = 0
        self._last_ms = 0.0
        self._busy = False
        self._last_status = "none"

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="qr-worker", daemon=True)
        self._thread.start()

    def submit(self, frame: np.ndarray):
        if not self._running:
            return
        with self._lock:
            self._submitted += 1
        try:
            self._input_q.put_nowait(frame.copy())
        except queue.Full:
            try:
                self._input_q.get_nowait()
                with self._lock:
                    self._dropped += 1
            except queue.Empty:
                pass
            try:
                self._input_q.put_nowait(frame.copy())
            except queue.Full:
                with self._lock:
                    self._dropped += 1

    def latest(self) -> QRResult:
        with self._lock:
            return self._latest

    def stats(self) -> dict:
        with self._lock:
            return {
                "submitted": self._submitted,
                "dropped": self._dropped,
                "processed": self._processed,
                "seen": self._seen,
                "decoded": self._decoded,
                "errors": self._errors,
                "last_ms": self._last_ms,
                "busy": self._busy,
                "last_status": self._last_status,
            }

    def clear(self):
        with self._lock:
            self._latest = QRResult()

    def reset_stats(self):
        with self._lock:
            self._submitted = 0
            self._dropped = 0
            self._processed = 0
            self._seen = 0
            self._decoded = 0
            self._errors = 0
            self._last_ms = 0.0
            self._busy = False
            self._last_status = "none"

    def stop(self):
        self._running = False
        try:
            self._input_q.put_nowait(None)
        except queue.Full:
            try:
                self._input_q.get_nowait()
                self._input_q.put_nowait(None)
            except queue.Empty:
                pass
        if self._thread is not None:
            self._thread.join(timeout=0.5)
            self._thread = None

    def _loop(self):
        while self._running:
            frame = self._input_q.get()
            if frame is None:
                break
            start = time.time()
            with self._lock:
                self._busy = True
            try:
                result = self._detector.detect(frame)
                error = False
            except Exception:
                result = QRResult()
                error = True
            elapsed_ms = (time.time() - start) * 1000.0
            with self._lock:
                self._processed += 1
                self._last_ms = elapsed_ms
                self._busy = False
                if error:
                    self._errors += 1
                    self._last_status = "error"
                elif result.decoded_text:
                    self._decoded += 1
                    self._last_status = "decoded"
                elif result.detected:
                    self._seen += 1
                    self._last_status = "seen_no_decode"
                else:
                    self._last_status = "none"
                self._latest = result


def load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as file_obj:
        return yaml.safe_load(file_obj)


class MissionController:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.log = init_logger(
            cfg["logging"]["log_path"],
            cfg["logging"]["level"],
            cfg["logging"]["console"],
        )
        self.vehicle: Optional[Vehicle] = None
        self.camera: Optional[Camera] = None
        self.guidance: Optional[DiveGuidance] = None

        self.state = "BOOT"
        self._takeoff_done = False
        self._transition_done = False
        self._kamikaze_requested = False
        self._last_log = 0.0
        self._cruise_heading_hold: Optional[float] = None
        self._attack_target: Optional[dict] = None
        self._attack_run_heading: Optional[float] = None
        self._dive_heading: Optional[float] = None
        self._last_distance_to_target: Optional[float] = None
        self._min_distance_to_target: Optional[float] = None
        self._dive_pitch_i = 0.0
        self._last_dive_tick_time: Optional[float] = None
        self._overshoot_counter = 0
        self._fw_active = False  # FBWA modu aktifken True olur
        self._fw_setup_time: Optional[float] = None
        self._approach_started_at: Optional[float] = None
        self._turn_aligned_since: Optional[float] = None  # Yönelim kilitlendiğindeki zaman damgası
        self._pullup_transition_done = False
        self._dive_started_at: Optional[float] = None
        self._last_qr_seen_log = 0.0
        self._qr_seen_capture_count = 0
        self._perf_window_start = time.time()
        self._perf_frames = 0
        self._mission_success_text: str = ""
        self._last_qr_perf_log = 0.0

        cam_cfg = cfg["camera"]
        mission_cfg = cfg["mission"]
        gui_cfg = cfg["gui"]
        self.qr_detector = QRDetector(
            cam_cfg["frame_width"], cam_cfg["frame_height"],
            mission_cfg["target_area_margin_ratio"],
        )
        self.qr_worker = SimpleQRWorker(self.qr_detector)
        self.gui = GUI(
            cam_cfg["frame_width"], cam_cfg["frame_height"],
            self.qr_detector.target_area, gui_cfg["title"], gui_cfg["capture_dir"],
        )

    # ── setup ──────────────────────────────────────────────────────
    def setup(self) -> bool:
        vc = self.cfg["vehicle"]
        self.vehicle = Vehicle(vc["connection"], vc["connection_timeout"], vc["arm_timeout"])
        if not self.vehicle.connect():
            return False
        self.vehicle.configure_for_mission(
            vc["disable_arming_checks"], vc["disable_compass"],
            vc.get("airframe_profile"), vc.get("reboot_if_params_changed", False),
        )
        if not self._init_camera():
            return False
        self._init_attack_target()
        return True

    def _init_attack_target(self):
        mc = self.cfg["mission"]
        home = self.vehicle.get_home_position() if self.vehicle else None
        if home is None:
            return
        if mc.get("use_takeoff_position_as_target", True):
            self._attack_target = dict(home)
        else:
            lat, lon = mc.get("target_lat"), mc.get("target_lon")
            if lat is None or lon is None:
                self._attack_target = dict(home)
            else:
                self._attack_target = {"lat": float(lat), "lon": float(lon), "alt": 0.0}
        if self._attack_target:
            self.log.info("MISSION",
                f"attack target lat={self._attack_target['lat']:.7f} lon={self._attack_target['lon']:.7f}")

    def _init_camera(self) -> bool:
        cc = self.cfg["camera"]
        topics = list(dict.fromkeys([cc["primary_topic"], *cc.get("fallback_topics", [])]))
        for topic in topics:
            cam = Camera(topic, "front_camera", cc["frame_width"], cc["frame_height"])
            deadline = time.time() + cc["startup_timeout_s"]
            while time.time() < deadline:
                if cam.read() is not None:
                    self.camera = cam
                    self.log.info("CAMERA", f"camera ready: {topic}")
                    return True
                time.sleep(0.1)
            cam.close()
        self.log.error("CAMERA", "camera init failed")
        return False

    # ── main loop ──────────────────────────────────────────────────
    def run(self):
        if not self.setup():
            self.shutdown(); return
        if self.cfg["gui"]["enable"]:
            cv2.namedWindow(self.gui.title, cv2.WINDOW_NORMAL)
            self.gui.install_mouse_callback()
        self.qr_worker.start()
        self.state = "TAKEOFF"
        try:
            while True:
                loop_start = time.time()
                telem = self.vehicle.get_telemetry() if self.vehicle else {}
                frame = self.camera.read() if self.camera else None
                if frame is None:
                    frame = np.zeros((self.cfg["camera"]["frame_height"],
                                      self.cfg["camera"]["frame_width"], 3), dtype=np.uint8)
                target = self._attack_target
                dist = Vehicle.distance_between(telem, target) if target and telem.get("lat") else 0.0
                if self.state == "FW_DIVE":
                    self.qr_worker.submit(frame)
                    qr = self.qr_worker.latest()
                else:
                    self.qr_worker.clear()
                    qr = QRResult()
                self._tick(telem, target, dist, qr)

                if self.cfg["gui"]["enable"]:
                    ready = self.state == "CRUISE_FORWARD"
                    overlay = self.gui.render(frame, self.state, telem, qr, dist,
                        self.guidance.dive_start_distance_m if self.guidance else 0.0, ready,
                        self._mission_success_text)
                    cv2.imshow(self.gui.title, overlay)
                    ev = self.gui.pop_events()
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord("q")): break
                    if ev.start_requested or key == ord("k"):
                        self._request_kamikaze(telem)
                if self.state == "SHUTDOWN": break
                now = time.time()
                self._perf_frames += 1
                if self.state == "FW_DIVE" and now - self._perf_window_start >= 1.0:
                    fps = self._perf_frames / max(now - self._perf_window_start, 0.001)
                    stats = self.qr_worker.stats()
                    qr_status = (
                        f"qr_ms={stats['last_ms']:.1f} qr_busy={stats['busy']} "
                        f"qr_sub={stats['submitted']} qr_proc={stats['processed']} "
                        f"qr_drop={stats['dropped']} qr_seen={stats['seen']} "
                        f"qr_ok={stats['decoded']} qr_err={stats['errors']} "
                        f"qr_last={stats['last_status']}"
                    )
                    self.log.info("PERF", f"fps={fps:.1f} {qr_status}", debounce=0.5)
                    self._perf_window_start = now
                    self._perf_frames = 0
                if now - self._last_log >= 3.0:
                    self.log.info("TELEM",
                        f"state={self.state} alt={telem.get('alt',0):.1f} "
                        f"hdg={telem.get('heading',0):.1f} dist={dist:.1f} "
                        f"mode={telem.get('mode','-')}")
                    self._last_log = now
                time.sleep(0.02)
        finally:
            self.shutdown()

    # ── state machine ──────────────────────────────────────────────
    def _tick(self, telem, target, dist, qr):
        s = self.state
        if s == "TAKEOFF":           self._do_takeoff()
        elif s == "TRANSITION":      self._do_transition()
        elif s == "CRUISE_FORWARD":  self._do_cruise(telem)
        elif s == "TURN_TO_TARGET":  self._do_turn(telem, target)
        elif s == "FW_APPROACH":     self._do_fw_approach(telem, target, dist)
        elif s == "FW_DIVE":         self._do_fw_dive(telem, target, dist, qr)
        elif s == "PULLUP":          self._do_pullup(telem)

    # ── TAKEOFF (GUIDED) ──────────────────────────────────────────
    def _do_takeoff(self):
        if self._takeoff_done: return
        alt = self.cfg["vehicle"]["takeoff_altitude_m"]
        self.log.info("MISSION", f"takeoff target={alt:.0f}m")
        self._takeoff_done = True
        if self.vehicle.arm_and_takeoff(
            alt,
            completion_ratio=float(self.cfg["vehicle"].get("takeoff_complete_ratio", 0.98)),
        ):
            self._init_attack_target()
            self.state = "TRANSITION"
        else:
            self.state = "SHUTDOWN"

    # ── TRANSITION → cruise setup ─────────────────────────────────
    def _do_transition(self):
        if self._transition_done: return
        self._transition_done = True
        t = self._attack_target
        if not t:
            self._init_attack_target()
            t = self._attack_target
        if not t:
            self.log.error("MISSION", "no attack target"); self.state = "SHUTDOWN"; return
        mc = self.cfg["mission"]
        self.guidance = DiveGuidance(
            t["lat"], t["lon"], self.cfg["vehicle"]["cruise_altitude_m"],
            mc["dive_recovery_altitude_m"], mc["dive_angle_deg"],
            mc["pullup_target_altitude_m"], mc["extra_dive_margin_m"],
            mc["approach_distance_m"], mc["transition_lead_distance_m"],
            target_offset_m=float(mc.get("target_offset_m", 8.0)),
            effective_dive_angle_deg=float(mc.get("effective_dive_angle_deg", 18.0)),
        )
        self.state = "CRUISE_FORWARD"
        self.log.info("MISSION", "cruise active; press K to arm kamikaze")

    # ── CRUISE_FORWARD (GUIDED VTOL — constant altitude forward leg) ──
    def _do_cruise(self, telem):
        hdg = float(telem.get("heading", 0))
        if self._cruise_heading_hold is None:
            self._cruise_heading_hold = hdg
        self._ensure_guided()
        self.vehicle.command_vtol_takeoff(self.cfg["vehicle"]["cruise_altitude_m"], min_interval_s=0.5)
        self._send_heading_waypoint(telem, self._cruise_heading_hold, self.cfg["vehicle"]["default_forward_leg_m"])

    # ── K pressed → kamikaze request ──────────────────────────────
    def _request_kamikaze(self, telem):
        if self.state != "CRUISE_FORWARD":
            self.log.warn("MISSION", "kamikaze ignored; wrong state", debounce=1.0); return
        self._mission_success_text = ""
        self._kamikaze_requested = True
        self._last_distance_to_target = None
        self._min_distance_to_target = None
        self._overshoot_counter = 0
        self._fw_active = False
        self._fw_setup_time = None
        self._approach_started_at = None
        self._pullup_transition_done = False
        # U dönüşü için FBWA seyirden GUIDED moduna geç
        self.vehicle.clear_rc_overrides()
        self._ensure_guided()
        dist = Vehicle.distance_between(telem, self._attack_target) if self._attack_target else 0
        self.log.info("MISSION", f"kamikaze armed dist={dist:.0f}m; turning to target")
        self.state = "TURN_TO_TARGET"

    # ── TURN_TO_TARGET (GUIDED → face QR coordinate) ──────────────
    def _do_turn(self, telem, target):
        if not target:
            self.state = "SHUTDOWN"; return
        self._ensure_guided()
        # Seyir irtifasında hedefe doğru düz uçuşla yönel
        self.vehicle.goto(target["lat"], target["lon"],
                          self.cfg["vehicle"]["cruise_altitude_m"],
                          airspeed=self.cfg["vehicle"]["cruise_speed_mps"])
        self.vehicle.command_vtol_takeoff(self.cfg["vehicle"]["cruise_altitude_m"], min_interval_s=0.5)

        bearing = Vehicle.bearing_to(telem, target)
        hdg = float(telem.get("heading", 0))
        err = ((bearing - hdg + 540) % 360) - 180
        dist = Vehicle.distance_between(telem, target)
        self.log.info("TURN",
            f"hdg={hdg:.0f} bearing={bearing:.0f} err={err:.1f} dist={dist:.0f}",
            debounce=1.0)

        # Yönelim tolerans içinde ve 1.5 saniye kararlı olmalı
        tol = self.cfg["mission"]["heading_tolerance_deg"]
        if abs(err) <= tol:
            if self._turn_aligned_since is None:
                self._turn_aligned_since = time.time()
                self.log.info("TURN", f"heading aligned, waiting for stability...")
            elif (time.time() - self._turn_aligned_since) >= 1.5:
                self._attack_run_heading = bearing
                self._dive_heading = bearing
                self._fw_active = False
                self._fw_setup_time = None
                self._approach_started_at = None
                self._turn_aligned_since = None
                self.state = "FW_APPROACH"
                self.log.info("MISSION",
                    f"ALIGNED STABLE hdg={hdg:.0f} bearing={bearing:.0f} dist={dist:.0f}; FW approach")
        else:
            # Yönelim bozuldu, zamanlayıcıyı sıfırla
            self._turn_aligned_since = None

    # ── FW_APPROACH (GUIDED until arm distance, FBWA only for dive entry) ───────
    def _do_fw_approach(self, telem, target, dist):
        if not target:
            self.state = "SHUTDOWN"; return
        alt = float(telem.get("alt", 0))
        vc = self.cfg["vehicle"]
        mc = self.cfg["mission"]
        pre_dive_target_alt = vc["cruise_altitude_m"] + float(mc.get("pre_dive_altitude_boost_m", 18.0))
        dive_start_dist = self._compute_dive_trigger_distance(alt)
        fw_arm_window = float(mc.get("final_fw_arm_window_m", 30.0))
        fw_arm_dist = max(
            dive_start_dist + fw_arm_window,
            float(mc.get("fixed_wing_dive_arm_distance_m", 160.0)),
        )
        # Yaklaşma sırasında yanal sapmayı sıfırlamak için hedef açısını dinamik hesapla
        attack_heading = Vehicle.bearing_to(telem, target)
        self._attack_run_heading = attack_heading

        # Dalış kapısına kadar VTOL/GUIDED modunda kal (irtifa kaybını engellemek için)
        if dist > fw_arm_dist:
            self._ensure_guided()
            self.vehicle.command_vtol_takeoff(pre_dive_target_alt, min_interval_s=0.5)
            self._send_heading_waypoint(telem, attack_heading, vc["default_forward_leg_m"], target_alt=pre_dive_target_alt)
            hdg = float(telem.get("heading", 0))
            hdg_err = ((attack_heading - hdg + 540) % 360) - 180
            self.log.info(
                "GUIDED_APPROACH",
                f"attack_hdg={attack_heading:.0f} hdg={hdg:.0f} err={hdg_err:.1f} dist={dist:.0f} alt={alt:.0f} trig={dive_start_dist:.0f}",
                debounce=0.5,
            )
            self._fw_active = False
            self._fw_setup_time = None
            return

        # Adım 1: Dalış girişine yakınken sabit kanata geçiş yap
        if not self._fw_active:
            self.log.info("MISSION", "FW transition for approach")
            self.vehicle.set_mode("FBWA")
            self.vehicle.set_rc_overrides(throttle_pwm=vc["fw_throttle_pwm"])
            self._fw_setup_time = time.time()
            self._fw_active = True
            self._approach_started_at = time.time()
            return

        # Adım 2: Geçiş sonrası kısa stabilizasyon süresi
        stabilize_s = max(0.5, min(float(vc.get("fixed_wing_stabilize_after_transition_s", 1.0)), 1.5))
        if self._fw_setup_time and (time.time() - self._fw_setup_time) < stabilize_s:
            self.vehicle.set_rc_overrides(
                roll_pwm=1500, pitch_pwm=1500,
                throttle_pwm=vc["fw_throttle_pwm"], yaw_pwm=1500)
            return

        # Adım 3: Son yaklaşmada FBWA modunda yönelim ve irtifayı koru
        hdg = float(telem.get("heading", 0))
        hdg_err = ((attack_heading - hdg + 540) % 360) - 180
        self._hold_fw_heading(telem, attack_heading, pre_dive_target_alt,
                               mc["dive_entry_speed_mps"])
        self.log.info("FW_APPROACH",
            f"attack_hdg={attack_heading:.0f} hdg={hdg:.0f} err={hdg_err:.1f} dist={dist:.0f} alt={alt:.0f} trig={dive_start_dist:.0f}",
            debounce=1.0)

        # Adım 4: Tetikleme mesafesinde hemen dalışı başlat
        if dist <= dive_start_dist:
            self._dive_heading = attack_heading
            self._last_distance_to_target = dist
            self._min_distance_to_target = dist
            self._dive_started_at = time.time()
            self._perf_window_start = time.time()
            self._perf_frames = 0
            self.qr_worker.clear()
            self.qr_worker.reset_stats()
            self.state = "FW_DIVE"
            self.log.info("MISSION",
                f"DIVE START dist={dist:.0f}m trigger={dive_start_dist:.0f}m alt={alt:.0f}m attack_hdg={attack_heading:.0f}")
            self._command_fw_dive(telem, target, dist)
            return

        # Emniyet: Yaklaşma zaman aşımı
        if self._approach_started_at and (time.time() - self._approach_started_at) > 120.0:
            self.log.warn("MISSION", "approach timeout; aborting")
            self._abort_to_guided(telem, "APPROACH_TIMEOUT")

    # ── FW_DIVE (FBWA, nose-down 30° via pitch RC override) ───────
    def _do_fw_dive(self, telem, target, dist, qr):
        alt = float(telem.get("alt", 0))
        vc = self.cfg["vehicle"]
        mc = self.cfg["mission"]
        pullup_alt = float(mc["dive_recovery_altitude_m"])

        # ── Mandatory pull-up gate ──
        if alt <= pullup_alt:
            self.log.warn("MISSION", f"PULLUP_ALT_LIMIT alt={alt:.1f}m limit={pullup_alt:.1f}m; VTOL RECOVERY")
            self._abort_to_guided(telem, "PULLUP_ALT_LIMIT")
            return

        # ── QR kod görüldü ama okunamadıysa geçici görsel kaydet ──
        if qr.detected and not qr.decoded_text:
            now = time.time()
            if now - self._last_qr_seen_log >= 0.5:
                bbox = qr.bbox.tolist() if qr.bbox is not None else []
                capture_path = self._save_qr_seen_frame(qr, alt, dist)
                self.log.info(
                    "QR",
                    f"QR_SEEN_NO_DECODE alt={alt:.1f}m dist={dist:.1f} bbox={bbox} in_area={qr.in_target_area} "
                    f"size_ok={qr.meets_size_requirement} capture={capture_path or '-'}",
                )
                self._last_qr_seen_log = now

        # ── QR kod başarıyla çözüldüyse görevi sonlandır ve tırmanışa geç ──
        if qr.decoded_text:
            self._mission_success_text = f"GOREV BASARILI QR: {qr.decoded_text}"
            capture_path = self._save_qr_success_frame(qr, alt, dist)
            self.log.info(
                "MISSION",
                f"QR_SUCCESS text='{qr.decoded_text}' alt={alt:.1f}m dist={dist:.1f}m capture={capture_path or '-'}",
            )
            self._abort_to_guided(telem, "QR_SUCCESS")
            return

        # ── Hedefi aşma / ıskalama durumunda tırmanışa geç ──
        if self._check_passed_target(dist):
            self.log.warn("MISSION", f"PULLUP_TARGET_PASSED dist={dist:.1f}m; VTOL RECOVERY")
            self._abort_to_guided(telem, "PULLUP_TARGET_PASSED")
            return

        self._command_fw_dive(telem, target, dist)

    def _do_pullup(self, telem):
        if not self._pullup_transition_done:
            self.vehicle.transition_to_guided_vtol(self.cfg["mission"]["pullup_target_altitude_m"])
            self._pullup_transition_done = True
        self._ensure_guided()
        alt = float(telem.get("alt", 0))
        hdg = float(self._dive_heading or telem.get("heading", 0))

        # Tırmanma komutu
        self.vehicle.command_vtol_takeoff(self.cfg["mission"]["pullup_target_altitude_m"], min_interval_s=0.5)

        # İlerlemeyi sürdürmek için ileri yönlü waypoint gönder
        self._send_heading_waypoint(telem, hdg, self.cfg["vehicle"]["default_forward_leg_m"])

        if alt >= self.cfg["mission"]["pullup_target_altitude_m"] * 0.9:
            self._reset_attack()
            self._cruise_heading_hold = hdg
            self.state = "CRUISE_FORWARD"
            self.log.info("MISSION", f"pullup complete alt={alt:.0f}m; cruise")

    # ── helpers ────────────────────────────────────────────────────
    def _ensure_guided(self):
        if self.vehicle is None: return
        mode = str(self.vehicle.get_telemetry().get("mode", "")).upper()
        if mode != "GUIDED":
            self.vehicle.clear_rc_overrides()
            self.vehicle.set_mode("GUIDED")
            self._fw_active = False

    def _abort_to_guided(self, telem, reason="PULLUP"):
        # Tırmanış için dalış modundan GUIDED moduna geçer
        self.vehicle.clear_rc_overrides()
        self.vehicle.transition_to_guided_vtol(self.cfg["mission"]["pullup_target_altitude_m"])
        self._fw_active = False
        self._pullup_transition_done = True
        self.state = "PULLUP"
        self.log.info("MISSION", f"abort -> PULLUP reason={reason} alt={telem.get('alt',0):.0f}m")

    def _hold_fw_heading(self, telem, target_hdg, target_alt, airspeed):
        # FBWA modunda yönelim ve irtifayı korumak için RC override komutları gönderir
        hdg = float(telem.get("heading", 0))
        alt = float(telem.get("alt", target_alt))
        vc = self.cfg["vehicle"]

        hdg_err = ((target_hdg - hdg + 540) % 360) - 180
        alt_err = target_alt - alt  # pozitif = tırmanması gerekiyor

        roll_sign = -1.0 if vc.get("fixed_wing_invert_roll", False) else 1.0
        pitch_sign = -1.0 if vc.get("fixed_wing_invert_pitch", True) else 1.0
        roll_limit = vc.get("fixed_wing_roll_limit_pwm", 140)
        pitch_limit = vc.get("fixed_wing_pitch_limit_pwm", 120)

        # Yön düzeltme için roll komutu (düz takip için yüksek kazanç)
        roll_cmd = np.clip(hdg_err * 5.0, -roll_limit, roll_limit)
        roll_pwm = int(1500 + roll_sign * roll_cmd)

        # İrtifa düzeltme komutu
        pitch_cmd = np.clip(alt_err * 4.5, -pitch_limit, pitch_limit)
        pitch_pwm = int(1500 + pitch_sign * pitch_cmd)

        # İrtifa farkına göre gaz (throttle) artırımı
        base_throttle = vc["fw_throttle_pwm"]
        if alt_err > 12:
            base_throttle = min(base_throttle + 100, 1850)
        elif alt_err > 4:
            base_throttle = min(base_throttle + 50, 1800)

        self.vehicle.set_rc_overrides(
            roll_pwm=roll_pwm,
            pitch_pwm=pitch_pwm,
            throttle_pwm=base_throttle,
            yaw_pwm=1500,
        )

    def _send_heading_waypoint(self, telem, hdg_deg, distance_m, target_alt=None):
        lat = float(telem.get("lat", 0))
        lon = float(telem.get("lon", 0))
        north = distance_m * math.cos(math.radians(hdg_deg))
        east = distance_m * math.sin(math.radians(hdg_deg))
        t_lat, t_lon = Vehicle.offset_position(lat, lon, north, east)
        if target_alt is None:
            target_alt = self.cfg["vehicle"]["cruise_altitude_m"]
        self.vehicle.command_guided_waypoint(
            t_lat, t_lon, target_alt,
            self.cfg["vehicle"]["cruise_speed_mps"],
            self.cfg["vehicle"]["command_period_s"],
        )

    def _compute_dive_trigger_distance(self, current_alt):
        mc = self.cfg["mission"]
        effective_angle = max(float(mc.get("effective_dive_angle_deg", 20.0)), 1.0)
        recovery_alt = float(mc["dive_recovery_altitude_m"])
        target_offset = float(mc.get("target_offset_m", 8.0))
        trigger_margin = float(mc.get("trigger_distance_margin_m", 8.0))
        trigger_extra = max(float(mc.get("dive_trigger_extra_distance_m", 0.0)), 0.0)
        vertical_delta = max(float(current_alt) - recovery_alt, 1.0)
        adaptive_trigger = target_offset + (vertical_delta / math.tan(math.radians(effective_angle))) + trigger_margin + trigger_extra
        return max(30.0, adaptive_trigger)

    def _check_overshoot(self, dist):
        if self._last_distance_to_target is None:
            self._last_distance_to_target = dist; return False
        delta = dist - self._last_distance_to_target
        self._last_distance_to_target = dist
        if delta > self.cfg["mission"]["overshoot_distance_delta_m"]:
            self._overshoot_counter += 1
        else:
            self._overshoot_counter = 0
        return self._overshoot_counter >= self.cfg["mission"]["overshoot_sample_limit"]

    def _check_passed_target(self, dist):
        if self._min_distance_to_target is None:
            self._min_distance_to_target = dist
            self._last_distance_to_target = dist
            return False
        self._min_distance_to_target = min(self._min_distance_to_target, dist)
        passed_margin = float(self.cfg["mission"].get("passed_target_margin_m", 3.0))
        passed = dist > (self._min_distance_to_target + passed_margin)
        self._last_distance_to_target = dist
        return passed

    def _reset_attack(self):
        self._kamikaze_requested = False
        self._dive_heading = None
        self._attack_run_heading = None
        self._last_distance_to_target = None
        self._min_distance_to_target = None
        self._dive_pitch_i = 0.0
        self._last_dive_tick_time = None
        self._overshoot_counter = 0
        self._fw_active = False
        self._fw_setup_time = None
        self._approach_started_at = None
        self._turn_aligned_since = None
        self._pullup_transition_done = False
        self._dive_started_at = None
        self._last_qr_seen_log = 0.0
        self._qr_seen_capture_count = 0

    def _save_qr_seen_frame(self, qr, alt, dist):
        # QR kodu görüldüğünde anlık kırpılmış görsel kaydeder
        if self._qr_seen_capture_count >= 6 or qr.bbox is None:
            return ""
        frame = qr.raw_frame if qr.raw_frame is not None else (self.camera.read() if self.camera else None)
        if frame is None:
            return ""
        x1, y1, x2, y2 = [int(round(v)) for v in qr.bbox.tolist()]
        h, w = frame.shape[:2]
        pad = 80
        x1 = max(0, x1 - pad)
        y1 = max(0, y1 - pad)
        x2 = min(w, x2 + pad)
        y2 = min(h, y2 + pad)
        if x2 <= x1 or y2 <= y1:
            return ""
        crop = frame[y1:y2, x1:x2]
        base_dir = os.path.dirname(os.path.abspath(__file__))
        capture_dir = self.cfg["gui"].get("capture_dir", "captures")
        if not os.path.isabs(capture_dir):
            capture_dir = os.path.join(base_dir, capture_dir)
        os.makedirs(capture_dir, exist_ok=True)
        path = os.path.join(
            capture_dir,
            f"qr_seen_{self._qr_seen_capture_count:02d}_alt{alt:.0f}_dist{dist:.0f}.jpg",
        )
        cv2.imwrite(path, crop)
        self._qr_seen_capture_count += 1
        return path

    def _save_qr_success_frame(self, qr, alt, dist):
        # QR kod başarıyla çözüldüğünde hem orijinal resmi hem de ön işlenmiş kesiti kaydeder
        frame = qr.raw_frame if qr.raw_frame is not None else (self.camera.read() if self.camera else None)
        if frame is None:
            return ""
        annotated = frame.copy()
        if qr.polygon is not None:
            pts = qr.polygon.astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(annotated, [pts], isClosed=True, color=(0, 255, 0), thickness=3)
        if qr.bbox is not None:
            x1, y1, x2, y2 = [int(round(v)) for v in qr.bbox.tolist()]
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (255, 0, 0), 2)
        cv2.putText(
            annotated,
            f"QR SUCCESS: {qr.decoded_text}",
            (30, 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            annotated,
            f"Altitude: {alt:.1f}m | Distance: {dist:.1f}m",
            (30, 85),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        base_dir = os.path.dirname(os.path.abspath(__file__))
        capture_dir = self.cfg["gui"].get("capture_dir", "captures")
        if not os.path.isabs(capture_dir):
            capture_dir = os.path.join(base_dir, capture_dir)
        os.makedirs(capture_dir, exist_ok=True)
        stamp = int(time.time() * 1000) % 100000
        path = os.path.join(
            capture_dir,
            f"qr_success_alt{alt:.0f}_dist{dist:.0f}_{stamp}.jpg",
        )
        cv2.imwrite(path, annotated)

        if qr.processed_frame is not None:
            proc_path = os.path.join(
                capture_dir,
                f"qr_success_processed_alt{alt:.0f}_dist{dist:.0f}_{stamp}.jpg",
            )
            cv2.imwrite(proc_path, qr.processed_frame)
            return f"{path} (processed: {proc_path})"

        return path

    def _command_fw_dive(self, telem, target, dist):
        vc = self.cfg["vehicle"]
        mc = self.cfg["mission"]

        if dist > 40.0 and target and "lat" in target and "lon" in target:
            dive_heading = Vehicle.bearing_to(telem, target)
            self._dive_heading = dive_heading
        else:
            dive_heading = float(
                self._dive_heading
                if self._dive_heading is not None
                else self._attack_run_heading
                if self._attack_run_heading is not None
                else telem.get("heading", 0.0)
            )
            self._dive_heading = dive_heading

        hdg = float(telem.get("heading", 0))
        hdg_err = ((dive_heading - hdg + 540) % 360) - 180
        roll_sign = -1.0 if vc.get("fixed_wing_invert_roll", False) else 1.0
        pitch_sign = -1.0 if vc.get("fixed_wing_invert_pitch", True) else 1.0
        roll_limit = int(mc.get("dive_roll_limit_pwm", 45))
        heading_gain = float(mc.get("dive_heading_gain", 1.2))
        dive_pitch_offset = int(mc.get("dive_pitch_offset_pwm", 400))
        dive_elapsed = time.time() - self._dive_started_at if self._dive_started_at else 999.0
        min_pitch_pwm = int(mc.get("dive_min_pitch_pwm", 1050))
        entry_pitch_floor_pwm = int(mc.get("dive_entry_pitch_floor_pwm", 1025))
        recovery_alt = float(mc.get("dive_recovery_altitude_m", 25.0))
        target_offset = float(mc.get("target_offset_m", 8.0))
        effective_angle = max(float(mc.get("effective_dive_angle_deg", 24.0)), 1.0)
        kp = float(mc.get("glide_slope_kp", 4.0))
        ki = float(mc.get("glide_slope_ki", 0.5))
        i_limit = float(mc.get("glide_slope_i_limit", 150.0))
        min_offset = int(mc.get("dive_min_pitch_offset_pwm", 260))
        max_offset = int(mc.get("dive_max_pitch_offset_pwm", 470))

        now = time.time()
        dt = now - (self._last_dive_tick_time if self._last_dive_tick_time is not None else now)
        self._last_dive_tick_time = now

        # Dalış aşamasında yanal yörünge hareketlerini engellemek için
        # sadece küçük yönelim düzeltmelerine izin verilir
        roll_pwm = int(1500 + roll_sign * np.clip(hdg_err * heading_gain, -roll_limit, roll_limit))
        ideal_alt = recovery_alt + max(float(dist) - target_offset, 0.0) * math.tan(math.radians(effective_angle))
        alt_error = float(telem.get("alt", ideal_alt)) - ideal_alt  # pozitif = süzülüş hattının üzerinde

        # Süzülüş hattı takibi için PI kontrolcü
        if dt > 0.0 and dt < 0.5:
            self._dive_pitch_i += alt_error * dt
        self._dive_pitch_i = np.clip(self._dive_pitch_i, -i_limit, i_limit)

        pi_correction = alt_error * kp + self._dive_pitch_i * ki
        dive_pitch_offset = int(np.clip(dive_pitch_offset - pi_correction, min_offset, max_offset))

        if dive_elapsed < 1.6:
            min_pitch_pwm = min(min_pitch_pwm, entry_pitch_floor_pwm)
            dive_pitch_offset = max(dive_pitch_offset, 450)
        pitch_pwm = int(np.clip(1500 + pitch_sign * dive_pitch_offset, min_pitch_pwm, 1900))
        target_speed = float(mc.get("dive_target_speed_mps", 14.0))
        current_speed = float(telem.get("groundspeed") or telem.get("airspeed") or target_speed)
        speed_error = target_speed - current_speed
        throttle = int(
            np.clip(
                float(mc.get("dive_throttle_pwm", 1100)) + speed_error * float(mc.get("dive_speed_gain_pwm", 18.0)),
                int(mc.get("dive_min_throttle_pwm", 1100)),
                int(mc.get("dive_max_throttle_pwm", 1250)),
            )
        )
        self.vehicle.set_airspeed(target_speed)

        self.vehicle.set_rc_overrides(
            roll_pwm=roll_pwm,
            pitch_pwm=pitch_pwm,
            throttle_pwm=throttle,
            yaw_pwm=1500,
        )
        self.log.debug(
            "DIVE",
            f"alt={float(telem.get('alt', 0)):.1f} dist={dist:.1f} dive_hdg={dive_heading:.1f} hdg_err={hdg_err:.1f} "
            f"ideal_alt={ideal_alt:.1f} alt_err={alt_error:.1f} spd={current_speed:.1f}->{target_speed:.1f} rc=({roll_pwm},{pitch_pwm},{throttle})",
        )

    def shutdown(self):
        self.log.info("MISSION", "shutdown")
        self.qr_worker.stop()
        if self.camera: self.camera.close(); self.camera = None
        if self.vehicle:
            self.vehicle.clear_rc_overrides()
            self.vehicle.close(); self.vehicle = None
        if self.cfg["gui"]["enable"]:
            try: cv2.destroyAllWindows()
            except: pass


def main():
    d = os.path.dirname(os.path.abspath(__file__))
    os.chdir(d)
    cfg_path = sys.argv[1] if len(sys.argv) > 1 else "config.yaml"
    MissionController(load_config(cfg_path)).run()

if __name__ == "__main__":
    main()
