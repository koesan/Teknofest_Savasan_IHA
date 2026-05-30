"""
Savaşan İHA - Single-Target EKF Tracker

Tek hedef, Extended Kalman Filter tabanlı tracker.
State: [cx, cy, vx, vy, w, h]
Multi-target tracking KULLANMIYORUZ - yarışmada tek hedef İHA var.
Bu sayede ID switching problemi tamamen ortadan kalkıyor.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Tuple

import numpy as np

from core.logger import get_logger


class TrackState(Enum):
    """Tracker durumları."""
    TENTATIVE = auto()   # İlk tespit, henüz onaylanmadı
    CONFIRMED = auto()   # Onaylandı, aktif takip
    COASTING = auto()    # Tespit yok ama tahmin devam ediyor
    LOST = auto()        # Hedef kayboldu


@dataclass
class TrackResult:
    """Tracker çıktısı."""
    state: TrackState
    centroid: Tuple[float, float]
    velocity: Tuple[float, float]  # px/s
    bbox: np.ndarray               # [x1, y1, x2, y2]
    confidence: float
    age: float                     # track yaşı (saniye)
    coast_time: float              # son tespitsiz süre
    hits: int                      # toplam tespit sayısı


class SingleTargetTracker:
    """
    Tek hedefli EKF tracker.

    Tasarım özellikleri:
    - Sadece tek hedef takibi → ID değişimi veya veri ilişkilendirme karmaşıklığı yoktur.
    - Sabit hızlı + boyut modelli EKF.
    - Hız tahmini ile yumuşak geçişli coasting (tahmin).
    - Mesafe eşikli gating (hatalı/gürültülü tespitleri reddetme).
    """

    def __init__(
        self,
        process_noise_pos: float = 8.0,
        process_noise_vel: float = 15.0,
        process_noise_size: float = 3.0,
        measurement_noise_pos: float = 4.0,
        measurement_noise_size: float = 8.0,
        gate_threshold: float = 50.0,
        min_hits_to_confirm: int = 3,
        max_coast_frames: int = 15,
        max_coast_time: float = 0.8,
        min_bbox_size: int = 6,
        frame_width: int = 1280,
        frame_height: int = 720,
    ):
        self._log = get_logger()

        # Yapılandırma
        self._process_noise_pos = process_noise_pos
        self._process_noise_vel = process_noise_vel
        self._process_noise_size = process_noise_size
        self._meas_noise_pos = measurement_noise_pos
        self._meas_noise_size = measurement_noise_size
        self._gate_threshold = gate_threshold
        self._min_hits = min_hits_to_confirm
        self._max_coast_frames = max_coast_frames
        self._max_coast_time = max_coast_time
        self._min_bbox_size = min_bbox_size
        self._frame_width = frame_width
        self._frame_height = frame_height
        self._confirm_min_area = float(max(min_bbox_size * min_bbox_size * 1.5, 30.0))
        self._confirm_min_conf = 0.15
        self._max_edge_margin_ratio = 0.03
        self._growth_snap_ratio = 1.15
        self._growth_blend = 0.95
        self._shrink_blend = 0.12

        # EKF durumu: [cx, cy, vx, vy, w, h]
        self._x: Optional[np.ndarray] = None  # durum vektörü (6,)
        self._P: Optional[np.ndarray] = None  # kovaryans matrisi (6,6)

        # Takip metaverileri
        self._state = TrackState.LOST
        self._hits = 0
        self._coast_frames = 0
        self._last_detection_time: Optional[float] = None
        self._track_start_time: Optional[float] = None
        self._last_update_time: Optional[float] = None
        self._last_confidence = 0.0
        self._last_gate = gate_threshold
        self._last_valid_result: Optional[TrackResult] = None

    @property
    def track_state(self) -> TrackState:
        return self._state

    @property
    def is_active(self) -> bool:
        return self._state in (TrackState.TENTATIVE, TrackState.CONFIRMED, TrackState.COASTING)

    def reset(self):
        """Tracker'ı tamamen sıfırla."""
        self._x = None
        self._P = None
        self._state = TrackState.LOST
        self._hits = 0
        self._coast_frames = 0
        self._last_detection_time = None
        self._track_start_time = None
        self._last_update_time = None
        self._last_confidence = 0.0
        self._last_valid_result = None
        self._log.info("TRACK", "Tracker reset")

    def _initialize(self, cx: float, cy: float, w: float, h: float, now: float):
        """Yeni track başlat."""
        self._x = np.array([cx, cy, 0.0, 0.0, w, h], dtype=np.float64)
        self._P = np.diag([
            self._meas_noise_pos ** 2,
            self._meas_noise_pos ** 2,
            self._process_noise_vel ** 2,
            self._process_noise_vel ** 2,
            self._meas_noise_size ** 2,
            self._meas_noise_size ** 2,
        ])
        self._state = TrackState.TENTATIVE
        self._hits = 1
        self._coast_frames = 0
        self._last_detection_time = now
        self._track_start_time = now
        self._last_update_time = now
        self._log.info("TRACK", f"NEW TRACK: pos=({cx:.0f},{cy:.0f}) size=({w:.0f}x{h:.0f}) state=TENTATIVE")

    def _predict(self, dt: float):
        """EKF predict adımı."""
        if self._x is None:
            return

        # State transition: constant velocity model
        # x_new = x + vx * dt, y_new = y + vy * dt
        F = np.eye(6)
        F[0, 2] = dt
        F[1, 3] = dt

        self._x = F @ self._x

        # Coasting sırasında velocity decay - çok büyümesini engelle
        if self._state == TrackState.COASTING:
            decay = 0.95  # Her adımda %5 azalt
            self._x[2] *= decay  # vx
            self._x[3] *= decay  # vy

        # Boyut sınırları
        self._x[4] = max(self._x[4], self._min_bbox_size)
        self._x[5] = max(self._x[5], self._min_bbox_size)

        # Process noise
        q_pos = self._process_noise_pos * dt
        q_vel = self._process_noise_vel * dt
        q_size = self._process_noise_size * dt
        Q = np.diag([q_pos**2, q_pos**2, q_vel**2, q_vel**2, q_size**2, q_size**2])

        self._P = F @ self._P @ F.T + Q

    def _gate_check(self, cx: float, cy: float, now: float = 0.0) -> bool:
        """Measurement gating - çok uzak tespitleri reddet.
        
        Velocity prediction ile daha akıllı gate check.
        """
        if self._x is None:
            return True  # İlk tespit, kabul et

        pred_cx, pred_cy = self._x[0], self._x[1]
        vx, vy = self._x[2], self._x[3]
        
        # Eğer son update'den beri zaman geçtiyse, velocity ile prediction yap
        if self._last_update_time is not None and now > 0:
            dt = now - self._last_update_time
            pred_cx += vx * dt
            pred_cy += vy * dt
        
        distance = np.hypot(cx - pred_cx, cy - pred_cy)
        
        # Dynamic gate: coasting sırasında gate'i kontrollü genişlet
        effective_gate = self._gate_threshold
        if self._state == TrackState.COASTING:
            effective_gate *= 1.8

        # Coasting süresine göre gate'i sınırlı artır
        coast_time = now - (self._last_detection_time or now) if self._last_detection_time else 0
        if coast_time > 1.0:
            effective_gate = max(effective_gate, self._gate_threshold + coast_time * 35)

        effective_gate = float(np.clip(effective_gate, self._gate_threshold, self._gate_threshold * 2.5))
        self._last_gate = effective_gate

        if distance > effective_gate:
            self._log.info("TRACK",
                f"GATE REJECT: det=({cx:.0f},{cy:.0f}) pred=({pred_cx:.0f},{pred_cy:.0f}) "
                f"dist={distance:.0f}>{effective_gate:.0f} vel=({vx:.0f},{vy:.0f}) "
                f"state={self._state.name} coast={coast_time:.2f}s")
            return False
        
        self._log.debug("TRACK",
            f"GATE OK: det=({cx:.0f},{cy:.0f}) pred=({pred_cx:.0f},{pred_cy:.0f}) "
            f"dist={distance:.0f}<{effective_gate:.0f}", debounce=0.1)
        return True

    def _update_ekf(self, cx: float, cy: float, w: float, h: float):
        """EKF update adımı."""
        if self._x is None:
            return

        prev_w = float(self._x[4])
        prev_h = float(self._x[5])
        prev_area = max(prev_w * prev_h, 1.0)
        meas_area = max(w * h, 1.0)

        # Measurement: [cx, cy, w, h]
        z = np.array([cx, cy, w, h])
        H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
        ], dtype=np.float64)

        R = np.diag([
            self._meas_noise_pos ** 2,
            self._meas_noise_pos ** 2,
            self._meas_noise_size ** 2,
            self._meas_noise_size ** 2,
        ])

        y = z - H @ self._x  # innovation
        S = H @ self._P @ H.T + R  # innovation covariance
        K = self._P @ H.T @ np.linalg.inv(S)  # Kalman gain

        self._x = self._x + K @ y
        I6 = np.eye(6)
        self._P = (I6 - K @ H) @ self._P

        area_growth = meas_area / prev_area
        width_growth = w / max(prev_w, 1.0)
        height_growth = h / max(prev_h, 1.0)

        # Yakınlaşırken dedektör bbox'ı bir anda büyür; EKF boyutu çok yavaş izlerse
        # lock ve mesafe kontrolü "hala uzak" sanıp gereksiz hızlanır.
        if (
            area_growth >= self._growth_snap_ratio
            or width_growth >= self._growth_snap_ratio
            or height_growth >= self._growth_snap_ratio
        ):
            blended_w = prev_w + (w - prev_w) * self._growth_blend
            blended_h = prev_h + (h - prev_h) * self._growth_blend
            self._x[4] = max(blended_w, self._min_bbox_size)
            self._x[5] = max(blended_h, self._min_bbox_size)
            self._P[4, 4] = max(self._P[4, 4], self._meas_noise_size ** 2)
            self._P[5, 5] = max(self._P[5, 5], self._meas_noise_size ** 2)
            self._log.info(
                "TRACK",
                f"SIZE SNAP: meas=({w:.0f}x{h:.0f}) prev=({prev_w:.0f}x{prev_h:.0f}) "
                f"new=({self._x[4]:.0f}x{self._x[5]:.0f}) growth={area_growth:.1f}x",
                debounce=0.1,
            )
        elif meas_area < prev_area * 0.85:
            self._x[4] = max(prev_w + (self._x[4] - prev_w) * self._shrink_blend, self._min_bbox_size)
            self._x[5] = max(prev_h + (self._x[5] - prev_h) * self._shrink_blend, self._min_bbox_size)

        # Boyut sınırları
        self._x[4] = max(self._x[4], self._min_bbox_size)
        self._x[5] = max(self._x[5], self._min_bbox_size)

    def _is_detection_credible(self, bbox, confidence: float, has_active_track: bool = False) -> bool:
        """Aşırı küçük veya frame kenarında kırpılmış kutuları erken ele."""
        x1, y1, x2, y2 = [float(v) for v in bbox]
        w = max(x2 - x1, 0.0)
        h = max(y2 - y1, 0.0)
        area = w * h
        frame_w = float(self._frame_width)
        frame_h = float(self._frame_height)
        edge_margin_x = frame_w * self._max_edge_margin_ratio
        edge_margin_y = frame_h * self._max_edge_margin_ratio
        touches_edge = (
            x1 <= edge_margin_x
            or y1 <= edge_margin_y
            or x2 >= frame_w - edge_margin_x
            or y2 >= frame_h - edge_margin_y
        )

        if has_active_track:
            min_area = max(float(self._min_bbox_size * self._min_bbox_size * 1.5), 40.0)
            min_conf = max(self._confirm_min_conf - 0.10, 0.12)
        else:
            min_area = self._confirm_min_area
            min_conf = self._confirm_min_conf

        if area < min_area and confidence < min_conf:
            self._log.info(
                "TRACK",
                f"REJECT DET: small/weak bbox=({w:.0f}x{h:.0f}) area={area:.0f} conf={confidence:.2f}",
            )
            return False
        if touches_edge and area < min_area * (1.8 if not has_active_track else 0.9) and confidence < 0.35:
            self._log.info(
                "TRACK",
                f"REJECT DET: edge bbox=({w:.0f}x{h:.0f}) conf={confidence:.2f} "
                f"xy=({x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f})",
            )
            return False
        return True

    def _is_false_track(self) -> bool:
        """Track fiziksel olarak çökerse veya görüntüde sürünürse iptal et."""
        if self._x is None:
            return False
        cx, cy, vx, vy, w, h = [float(v) for v in self._x]
        area = w * h
        near_bottom = cy >= float(self._frame_height) * 0.90
        very_small = area <= max(self._confirm_min_area * 0.85, 160.0)
        nearly_static = abs(vx) + abs(vy) < 8.0
        if self._state == TrackState.COASTING and near_bottom and very_small and nearly_static:
            self._log.warn(
                "TRACK",
                f"FALSE TRACK DROP: pos=({cx:.0f},{cy:.0f}) size=({w:.0f}x{h:.0f}) "
                f"vel=({vx:.0f},{vy:.0f}) coast_frames={self._coast_frames}",
            )
            return True
        return False

    def update(
        self,
        detections: Optional[List[object] | object] = None,
        now: Optional[float] = None,
    ) -> TrackResult:
        """
        Ana update fonksiyonu. Birden fazla tespiti kabul eder ve en yakın
        olanı EKF tahminiyle eşleştirir (Nearest Neighbor Association).

        Args:
            detections: Tek bir Detection nesnesi veya Detection listesi (veya None)
            now: Zaman damgası

        Returns:
            TrackResult
        """
        now = now or time.time()

        if self._last_update_time is not None:
            dt = max(now - self._last_update_time, 1e-4)
        else:
            dt = 0.033  # ~30fps default

        # Girdiyi normalize et
        det_list = []
        if detections is not None:
            if isinstance(detections, list):
                det_list = detections
            else:
                det_list = [detections]

        # En uygun tespiti seç (Association & Gating)
        best_candidate = None
        best_dist = float("inf")
        best_conf = -1.0
        
        has_active_track = self._state in (TrackState.TENTATIVE, TrackState.CONFIRMED, TrackState.COASTING)

        for det in det_list:
            bbox = det.bbox
            cx = float((bbox[0] + bbox[2]) / 2)
            cy = float((bbox[1] + bbox[3]) / 2)
            w = max(float(bbox[2] - bbox[0]), self._min_bbox_size)
            h = max(float(bbox[3] - bbox[1]), self._min_bbox_size)
            conf = float(det.confidence)

            if not self._is_detection_credible(bbox, conf, has_active_track=has_active_track):
                continue

            if self._x is not None:
                # Gating kontrolü
                if self._gate_check(cx, cy, now):
                    # EKF tahminine olan mesafe (L2 norm)
                    pred_cx, pred_cy = self._x[0], self._x[1]
                    dist = np.hypot(cx - pred_cx, cy - pred_cy)
                    if dist < best_dist:
                        best_dist = dist
                        best_candidate = (cx, cy, w, h, conf)
            else:
                # Track yoksa, en yüksek güvenilirliğe sahip olanı seç
                if conf > best_conf:
                    best_conf = conf
                    best_candidate = (cx, cy, w, h, conf)

        # Seçilen tespit değerlerini aktar
        has_measurement = best_candidate is not None
        if has_measurement:
            meas_cx, meas_cy, meas_w, meas_h, confidence = best_candidate
        else:
            meas_cx = meas_cy = meas_w = meas_h = confidence = 0.0

        # --- State machine ---
        if self._state == TrackState.LOST:
            if has_measurement:
                self._initialize(meas_cx, meas_cy, meas_w, meas_h, now)
                self._last_confidence = confidence
            self._last_update_time = now
            return self._make_result(now)

        # Predict
        self._predict(dt)

        if has_measurement:
            # Update
            self._update_ekf(meas_cx, meas_cy, meas_w, meas_h)
            self._hits += 1
            self._coast_frames = 0
            self._last_detection_time = now
            self._last_confidence = confidence

            # State transition
            if self._state == TrackState.TENTATIVE:
                if self._hits >= self._min_hits:
                    self._state = TrackState.CONFIRMED
                    self._log.info("TRACK",
                        f"STATE CHANGE: TENTATIVE→CONFIRMED hits={self._hits} "
                        f"pos=({self._x[0]:.0f},{self._x[1]:.0f}) vel=({self._x[2]:.0f},{self._x[3]:.0f})")
            elif self._state == TrackState.COASTING:
                self._state = TrackState.CONFIRMED
                self._log.info("TRACK",
                    f"STATE CHANGE: COASTING→CONFIRMED (reacquired) "
                    f"pos=({self._x[0]:.0f},{self._x[1]:.0f})")
        else:
            # No measurement - coast
            self._coast_frames += 1
            coast_time = now - (self._last_detection_time or now)

            if self._state in (TrackState.CONFIRMED, TrackState.TENTATIVE):
                old_state = self._state
                self._state = TrackState.COASTING
                self._log.info("TRACK",
                    f"STATE CHANGE: {old_state.name}→COASTING frame={self._coast_frames} "
                    f"last_pos=({self._x[0]:.0f},{self._x[1]:.0f}) vel=({self._x[2]:.0f},{self._x[3]:.0f})")

            # Coasting sırasında confidence azalt ama hemen kaybetme
            if self._is_false_track() or self._coast_frames > self._max_coast_frames or coast_time > self._max_coast_time:
                old_state = self._state
                self._log.info("TRACK",
                    f"STATE CHANGE: {old_state.name}→LOST coast_frames={self._coast_frames} "
                    f"coast_time={coast_time:.2f}s last_pos=({self._x[0]:.0f},{self._x[1]:.0f}) "
                    f"vel=({self._x[2]:.0f},{self._x[3]:.0f})")
                self._state = TrackState.LOST
                self.reset()

        self._last_update_time = now
        return self._make_result(now)

    def _make_result(self, now: float) -> TrackResult:
        """Tracker sonucunu oluştur."""
        if self._x is None:
            return TrackResult(
                state=TrackState.LOST,
                centroid=(0.0, 0.0),
                velocity=(0.0, 0.0),
                bbox=np.array([0, 0, 0, 0], dtype=float),
                confidence=0.0,
                age=0.0,
                coast_time=0.0,
                hits=0,
            )

        cx, cy, vx, vy, w, h = self._x
        bbox = np.array([cx - w/2, cy - h/2, cx + w/2, cy + h/2], dtype=float)
        bbox[0] = np.clip(bbox[0], 0.0, float(self._frame_width - 1))
        bbox[1] = np.clip(bbox[1], 0.0, float(self._frame_height - 1))
        bbox[2] = np.clip(bbox[2], bbox[0] + 1.0, float(self._frame_width))
        bbox[3] = np.clip(bbox[3], bbox[1] + 1.0, float(self._frame_height))
        age = now - self._track_start_time if self._track_start_time else 0.0
        coast_time = now - self._last_detection_time if self._last_detection_time else 0.0

        # Coasting sırasında confidence azalsın
        conf = self._last_confidence
        if self._state == TrackState.COASTING:
            conf *= max(0.1, 1.0 - coast_time / self._max_coast_time)

        result = TrackResult(
            state=self._state,
            centroid=(float(cx), float(cy)),
            velocity=(float(vx), float(vy)),
            bbox=bbox,
            confidence=conf,
            age=age,
            coast_time=coast_time if self._state == TrackState.COASTING else 0.0,
            hits=self._hits,
        )
        if self._state in (TrackState.CONFIRMED, TrackState.COASTING):
            self._last_valid_result = result
        return result

    def get_last_valid_result(self) -> Optional[TrackResult]:
        """Kayıp öncesi son geçerli track sonucunu döndür."""
        return self._last_valid_result

    def get_predicted_position(self, dt_ahead: float) -> Optional[Tuple[float, float]]:
        """dt_ahead saniye sonraki tahmini pozisyon."""
        if self._x is None:
            return None
        cx, cy, vx, vy = self._x[0], self._x[1], self._x[2], self._x[3]
        return (cx + vx * dt_ahead, cy + vy * dt_ahead)

    def get_predicted_roi(
        self, dt_ahead: float = 0.0, expand: float = 2.5
    ) -> Optional[Tuple[int, int, int, int]]:
        """Predicted ROI for reacquisition search."""
        if self._x is None:
            return None
        cx = self._x[0] + self._x[2] * dt_ahead
        cy = self._x[1] + self._x[3] * dt_ahead
        w = max(self._x[4] * expand, self._frame_width * 0.22)
        h = max(self._x[5] * expand, self._frame_height * 0.22)
        return (
            int(np.clip(cx - w / 2, 0, self._frame_width - 1)),
            int(np.clip(cy - h / 2, 0, self._frame_height - 1)),
            int(np.clip(cx + w / 2, 1, self._frame_width)),
            int(np.clip(cy + h / 2, 1, self._frame_height)),
        )
