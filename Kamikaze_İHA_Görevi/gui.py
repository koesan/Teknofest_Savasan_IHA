from __future__ import annotations

import os
import time
from dataclasses import dataclass

import cv2
import numpy as np

from qr_detector import QRResult


@dataclass
class GUIEvent:
    start_requested: bool = False


class GUI:
    def __init__(self, frame_width: int, frame_height: int, target_area: tuple, title: str, capture_dir: str):
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.target_area = target_area
        self.title = title
        self.capture_dir = capture_dir
        self._button_rect = (frame_width - 270, frame_height - 80, frame_width - 20, frame_height - 25)
        self._event = GUIEvent()
        os.makedirs(capture_dir, exist_ok=True)

    def install_mouse_callback(self):
        cv2.setMouseCallback(self.title, self._on_mouse)

    def pop_events(self) -> GUIEvent:
        event = GUIEvent(start_requested=self._event.start_requested)
        self._event.start_requested = False
        return event

    def _on_mouse(self, event, x, y, flags, param):
        del flags, param
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        x1, y1, x2, y2 = self._button_rect
        if x1 <= x <= x2 and y1 <= y <= y2:
            self._event.start_requested = True

    def render(
        self,
        frame: np.ndarray,
        state: str,
        telemetry: dict,
        qr_result: QRResult,
        distance_to_target_m: float,
        dive_start_distance_m: float,
        target_ready: bool,
        success_text: str = "",
    ) -> np.ndarray:
        overlay = frame.copy()
        if overlay.size == 0:
            overlay = np.zeros((self.frame_height, self.frame_width, 3), dtype=np.uint8)

        panel = overlay.copy()
        cv2.rectangle(panel, (10, 10), (340, 225), (15, 15, 15), -1)
        cv2.rectangle(panel, (10, self.frame_height - 95), (self.frame_width - 10, self.frame_height - 10), (15, 15, 15), -1)
        overlay = cv2.addWeighted(panel, 0.55, overlay, 0.45, 0.0)

        lines = [
            f"STATE: {state}",
            f"ALT: {telemetry.get('alt', 0.0):.1f} m",
            f"HDG: {telemetry.get('heading', 0.0):.1f} deg",
            f"GS: {telemetry.get('groundspeed', 0.0):.1f} m/s",
            f"MODE: {telemetry.get('mode', '-')}",
            f"DIST: {distance_to_target_m:.1f} m",
            f"DIVE START: {dive_start_distance_m:.1f} m",
            f"QR READ: {'OK' if qr_result.decoded_text else 'NO'}",
            f"QR TEXT: {qr_result.decoded_text or '-'}",
        ]
        for idx, line in enumerate(lines):
            cv2.putText(overlay, line, (22, 34 + idx * 20), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (245, 245, 245), 1)


        now = time.time()
        stamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now)) + f".{int((now % 1) * 1000):03d}"
        cv2.putText(overlay, stamp, (self.frame_width - 320, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (245, 245, 245), 1)

        x1, y1, x2, y2 = self._button_rect
        button_color = (0, 170, 0) if target_ready else (70, 70, 70)
        cv2.rectangle(overlay, (x1, y1), (x2, y2), button_color, -1)
        cv2.rectangle(overlay, (x1, y1), (x2, y2), (255, 255, 255), 1)
        label = "START KAMIKAZE" if target_ready else "WAIT ALT/STATE"
        cv2.putText(overlay, label, (x1 + 18, y1 + 34), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)

        status_line = (
            "Coordinate based attack active. No visual centering."
            if state in {"TURN_TO_TARGET", "APPROACH", "DIVE", "PULLUP", "ABORT_CLIMB"}
            else "Cruise forward. Click button or press K to arm kamikaze."
        )
        cv2.putText(
            overlay,
            status_line,
            (18, self.frame_height - 42),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            2,
        )

        # Görev başarılı veya başarısız olduğunda ekran ortasına oyun tarzı HUD paneli çiz
        if success_text:
            h, w = overlay.shape[:2]
            cx, cy = w // 2, h // 2
            card_w, card_h = 520, 180
            cx1 = max(0, cx - card_w // 2)
            cy1 = max(0, cy - card_h // 2)
            cx2 = min(w, cx + card_w // 2)
            cy2 = min(h, cy + card_h // 2)

            sub_img = overlay[cy1:cy2, cx1:cx2]
            black_rect = np.zeros_like(sub_img)
            cv2.rectangle(black_rect, (0, 0), (cx2 - cx1, cy2 - cy1), (0, 40, 0), -1)
            cv2.addWeighted(sub_img, 0.25, black_rect, 0.75, 0, dst=sub_img)

            cv2.rectangle(overlay, (cx1, cy1), (cx2, cy2), (0, 255, 0), 3)

            text1 = "GOREV BASARILI"
            text2 = "QR Kod Okundu!"
            text3 = f"Metin: {success_text.replace('GOREV BASARILI QR: ', '')}"

            t1_size = cv2.getTextSize(text1, cv2.FONT_HERSHEY_SIMPLEX, 1.1, 3)[0]
            t2_size = cv2.getTextSize(text2, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
            t3_size = cv2.getTextSize(text3, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]

            cv2.putText(overlay, text1, (cx - t1_size[0] // 2, cy1 + 50), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 0), 3, cv2.LINE_AA)
            cv2.putText(overlay, text2, (cx - t2_size[0] // 2, cy1 + 95), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (240, 240, 240), 2, cv2.LINE_AA)
            cv2.putText(overlay, text3, (cx - t3_size[0] // 2, cy1 + 140), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 255, 200), 2, cv2.LINE_AA)

        elif state == "PULLUP":
            h, w = overlay.shape[:2]
            cx, cy = w // 2, h // 2
            card_w, card_h = 520, 150
            cx1 = max(0, cx - card_w // 2)
            cy1 = max(0, cy - card_h // 2)
            cx2 = min(w, cx + card_w // 2)
            cy2 = min(h, cy + card_h // 2)

            sub_img = overlay[cy1:cy2, cx1:cx2]
            black_rect = np.zeros_like(sub_img)
            cv2.rectangle(black_rect, (0, 0), (cx2 - cx1, cy2 - cy1), (0, 0, 40), -1)
            cv2.addWeighted(sub_img, 0.25, black_rect, 0.75, 0, dst=sub_img)

            cv2.rectangle(overlay, (cx1, cy1), (cx2, cy2), (0, 0, 255), 3)

            text1 = "GOREV BASARISIZ"
            text2 = "QR Kod Okunamadi!"

            t1_size = cv2.getTextSize(text1, cv2.FONT_HERSHEY_SIMPLEX, 1.1, 3)[0]
            t2_size = cv2.getTextSize(text2, cv2.FONT_HERSHEY_SIMPLEX, 0.75, 2)[0]

            cv2.putText(overlay, text1, (cx - t1_size[0] // 2, cy1 + 55), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 255), 3, cv2.LINE_AA)
            cv2.putText(overlay, text2, (cx - t2_size[0] // 2, cy1 + 105), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (220, 220, 220), 2, cv2.LINE_AA)

        return overlay
