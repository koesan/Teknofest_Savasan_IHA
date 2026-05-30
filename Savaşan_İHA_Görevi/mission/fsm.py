"""
Savaşan İHA - Mission Finite State Machine

Temiz, log'lanabilir durum makinesi.
Her state kendi enter/update/exit mantığına sahip.
"""

from __future__ import annotations

import time
from enum import Enum, auto
from typing import Callable, Dict, Optional

from core.logger import get_logger
from perception.tracker import TrackState


class MissionState(Enum):
    """Görev durumları."""
    INIT = auto()
    TAKEOFF = auto()
    SEARCH = auto()
    DETECTED = auto()
    INTERCEPT = auto()
    TRACK = auto()
    LOCK_HOLD = auto()
    LOCKED = auto()
    REACQUIRE = auto()
    LAND = auto()


class MissionFSM:
    """
    Görev durumu yönetimi.

    Geçişler:
    INIT → TAKEOFF → SEARCH → DETECTED → INTERCEPT → TRACK → LOCK_HOLD → LOCKED
                       ↑        ↑                       ↓
                       └────────┴──── REACQUIRE ←───────┘
    """

    def __init__(
        self,
        detection_confirm_time: float = 0.3,  # 0.5 -> 0.3: daha hızlı onay
        intercept_timeout: float = 120.0,  # 30 -> 120: daha uzun takip
        reacquire_timeout: float = 30.0,  # 8 -> 30: daha uzun arama
        track_loss_grace: float = 3.0,  # 2 -> 3: daha toleranslı
    ):
        self._log = get_logger()
        self._state = MissionState.INIT
        self._state_enter_time = 0.0
        self._last_now = 0.0
        self._prev_state: Optional[MissionState] = None

        # Yapılandırılabilir zaman aşımları
        self._detect_confirm = detection_confirm_time
        self._intercept_timeout = intercept_timeout
        self._reacquire_timeout = reacquire_timeout
        self._track_loss_grace = track_loss_grace

        # Takip durumu parametreleri
        self._first_detect_time: Optional[float] = None
        self._track_lost_time: Optional[float] = None
        self._total_locks = 0

    @property
    def state(self) -> MissionState:
        return self._state

    @property
    def state_duration(self) -> float:
        return max(self._last_now - self._state_enter_time, 0.0)

    @property
    def total_locks(self) -> int:
        return self._total_locks

    def _transition(self, new_state: MissionState, reason: str = ""):
        """Durum geçişi."""
        old = self._state
        self._prev_state = old
        self._state = new_state
        self._state_enter_time = self._last_now
        
        # Detaylı FSM geçiş logu
        self._log.info("FSM",
            f"TRANSITION: {old.name} → {new_state.name} "
            f"reason=\"{reason}\" duration={self.state_duration:.1f}s "
            f"track_state= track_lost={self._track_lost_time is not None}")

    def update(
        self,
        track_state: TrackState,
        has_detection: bool,
        lock_progress: float,
        is_locked: bool,
        now: Optional[float] = None,
    ) -> MissionState:
        """
        FSM güncelle.

        Parametreler:
            track_state: tracker durumu
            has_detection: bu karede tespit var mı
            lock_progress: lock zamanlayıcı ilerlemesi [0..1]
            is_locked: tam kilitlenme başarıldı mı
            now: zaman damgası

        Dönen:
            Mevcut durum
        """
        now = now or time.time()
        self._last_now = now
        dt = now - self._state_enter_time

        # =====================================================================
        # INIT
        # =====================================================================
        if self._state == MissionState.INIT:
            # Dışarıdan takeoff çağrıldığında geçiş yapılır
            pass

        # =====================================================================
        # TAKEOFF
        # =====================================================================
        elif self._state == MissionState.TAKEOFF:
            # Takeoff tamamlandığında dışarıdan transition çağrılır
            pass

        # =====================================================================
        # SEARCH
        # =====================================================================
        elif self._state == MissionState.SEARCH:
            if has_detection:
                self._first_detect_time = now
                self._transition(MissionState.DETECTED, "target_detected")

        # =====================================================================
        # DETECTED - kısa onay süresi
        # =====================================================================
        elif self._state == MissionState.DETECTED:
            if not has_detection:
                if dt > self._detect_confirm:
                    self._transition(MissionState.SEARCH, "detection_lost")
                    self._first_detect_time = None
            else:
                if track_state == TrackState.CONFIRMED:
                    self._transition(MissionState.INTERCEPT, "track_confirmed")
                elif track_state in (TrackState.TENTATIVE, TrackState.COASTING):
                    # Tracker çalışıyor, ilerle
                    if dt > self._detect_confirm:
                        self._transition(MissionState.INTERCEPT, "track_active")
                elif dt > self._detect_confirm * 3:
                    # Tracker onaylamasa bile ilerle (daha toleranslı)
                    self._transition(MissionState.INTERCEPT, "confirm_timeout")

        # =====================================================================
        # INTERCEPT - hedefe yanaşma
        # =====================================================================
        elif self._state == MissionState.INTERCEPT:
            if track_state == TrackState.LOST:
                self._track_lost_time = now
                self._transition(MissionState.REACQUIRE, "track_lost")
            elif lock_progress > 0.0:
                # Lock timer başladı → TRACK moduna geç
                self._transition(MissionState.TRACK, "lock_started")
            # Timeout'u kaldır - hedef bulunduysa aramaya dönme!
            # Sadece çok uzun süre lock_progress=0 kalırsa uyar

        # =====================================================================
        # TRACK - aktif takip ve lock hold
        # =====================================================================
        elif self._state == MissionState.TRACK:
            if is_locked:
                self._total_locks += 1
                self._transition(MissionState.LOCKED, "lock_achieved")
            elif track_state == TrackState.LOST:
                self._track_lost_time = now
                self._transition(MissionState.REACQUIRE, "track_lost")
            elif track_state == TrackState.COASTING:
                # Grace period
                pass

        # =====================================================================
        # LOCKED - kilitlenme başarılı
        # =====================================================================
        elif self._state == MissionState.LOCKED:
            # Kilitlenme tamamlandı - yeni hedef aramaya dön
            if dt > 2.0:
                self._transition(MissionState.SEARCH, "lock_complete_search_new")

        # =====================================================================
        # REACQUIRE - hedef kayıp, yeniden bul
        # =====================================================================
        elif self._state == MissionState.REACQUIRE:
            if has_detection and track_state in (TrackState.TENTATIVE, TrackState.CONFIRMED, TrackState.COASTING):
                self._transition(MissionState.INTERCEPT, "reacquired")
            elif track_state == TrackState.COASTING:
                # Coasting'de bile yeni detection gelmiş olabilir, kabul et
                self._transition(MissionState.INTERCEPT, "reacquired_coasting")
            elif dt > self._reacquire_timeout:
                self._transition(MissionState.SEARCH, "reacquire_timeout")

        # =====================================================================
        # LAND
        # =====================================================================
        elif self._state == MissionState.LAND:
            pass

        return self._state

    def trigger_takeoff_complete(self):
        """Kalkış tamamlandı."""
        if self._state == MissionState.TAKEOFF:
            self._transition(MissionState.SEARCH, "takeoff_complete")

    def trigger_takeoff(self):
        """Kalkış başlat."""
        if self._state == MissionState.INIT:
            self._transition(MissionState.TAKEOFF, "takeoff_initiated")

    def trigger_land(self):
        """İniş komutu."""
        self._transition(MissionState.LAND, "land_commanded")

    def get_status_string(self) -> str:
        """UI için durum string'i."""
        return (f"{self._state.name} "
                f"(t={self.state_duration:.1f}s locks={self._total_locks})")
