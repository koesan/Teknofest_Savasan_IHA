"""
Savaşan İHA - YOLO Detector Wrapper
model.pt modelini kullanan, temiz, çizim yapmayan dedektör.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np

from core.logger import get_logger


@dataclass
class Detection:
    """Tekil tespit sonucu."""
    bbox: np.ndarray          # [x1, y1, x2, y2]
    centroid: Tuple[float, float]
    confidence: float
    class_id: int
    class_name: str = ""
    area: float = 0.0

    def __post_init__(self):
        x1, y1, x2, y2 = self.bbox
        self.area = float((x2 - x1) * (y2 - y1))
        if self.centroid is None:
            self.centroid = (float((x1 + x2) / 2), float((y1 + y2) / 2))


class Detector:
    """YOLO tabanlı İHA dedektörü. Çizim yapılmaz - sorumlulukların ayrılması prensibi uygulanmıştır."""

    def __init__(
        self,
        model_path: str,
        confidence_threshold: float = 0.20,
        iou_threshold: float = 0.3,
        imgsz: int = 1280,
        max_detections: int = 5,
        target_classes: Optional[List[int]] = None,
    ):
        self._log = get_logger()

        # Model yolunu proje kök dizinine göre çözümle
        if not os.path.isabs(model_path):
            project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            model_path = os.path.join(project_root, model_path)

        if not os.path.exists(model_path):
            raise FileNotFoundError(f"YOLO model not found: {model_path}")

        from ultralytics import YOLO
        self._model = YOLO(model_path)
        self._conf = confidence_threshold
        self._iou = iou_threshold
        self._imgsz = imgsz
        self._max_det = max_detections
        self._classes = target_classes
        self._roi_verify_counter = 0
        self._log.info("DETECT", f"Model loaded: {model_path} conf={self._conf}")

    def detect(
        self,
        frame: np.ndarray,
        roi: Optional[Tuple[int, int, int, int]] = None,
        imgsz_override: Optional[int] = None,
    ) -> List[Detection]:
        """
        Görüntü (frame) veya ROI üzerinde tespit işlemini çalıştır.

        Parametreler:
            frame: BGR görüntüsü
            roi: İsteğe bağlı, tespitten önce kırpılacak alan koordinatları (x1, y1, x2, y2)
            imgsz_override: çıkarım (inference) çözünürlüğünü ezmek/değiştirmek için değer

        Dönen:
            Güven skoruna göre azalan sırada sıralanmış Detection (Tespit) listesi
        """
        if frame is None or frame.size == 0:
            return []

        # ROI kırpma
        offset_x, offset_y = 0, 0
        input_frame = frame
        if roi is not None:
            rx1, ry1, rx2, ry2 = roi
            h, w = frame.shape[:2]
            rx1, ry1 = max(0, rx1), max(0, ry1)
            rx2, ry2 = min(w, rx2), min(h, ry2)
            if rx2 - rx1 < 32 or ry2 - ry1 < 32:
                return []
            input_frame = frame[ry1:ry2, rx1:rx2]
            offset_x, offset_y = rx1, ry1

        # BGR formatında olduğundan emin ol
        if len(input_frame.shape) == 2:
            input_frame = cv2.cvtColor(input_frame, cv2.COLOR_GRAY2BGR)

        # Çıkarımı çalıştır
        results = self._model.predict(
            input_frame,
            conf=self._conf,
            iou=self._iou,
            classes=self._classes,
            max_det=self._max_det,
            imgsz=imgsz_override or self._imgsz,
            verbose=False,
        )

        detections: List[Detection] = []
        for result in results:
            if result.boxes is None or len(result.boxes) == 0:
                continue
            for box in result.boxes:
                bbox = box.xyxy[0].cpu().numpy().astype(float)
                # Koordinatları ana görüntü koordinat sistemine geri çevir
                bbox[0] += offset_x
                bbox[1] += offset_y
                bbox[2] += offset_x
                bbox[3] += offset_y

                cx = float((bbox[0] + bbox[2]) / 2)
                cy = float((bbox[1] + bbox[3]) / 2)

                det = Detection(
                    bbox=bbox,
                    centroid=(cx, cy),
                    confidence=float(box.conf),
                    class_id=int(box.cls),
                    class_name=result.names.get(int(box.cls), ""),
                )
                detections.append(det)

        # Güven skoruna göre sırala ve en iyisini al
        detections.sort(key=lambda d: d.confidence, reverse=True)
        
        # Tespitleri logla (log kalabalığını önlemek için sönümlenmiş şekilde)
        if detections:
            best = detections[0]
            w, h = best.bbox[2] - best.bbox[0], best.bbox[3] - best.bbox[1]
            self._log.debug("DETECT",
                f"DETECTED: {len(detections)} target(s) best_conf={best.confidence:.2f} "
                f"pos=({best.centroid[0]:.0f},{best.centroid[1]:.0f}) size=({w:.0f}x{h:.0f})",
                debounce=0.1)
        
        return detections

    def detect_with_roi_fallback(
        self,
        frame: np.ndarray,
        predicted_roi: Optional[Tuple[int, int, int, int]] = None,
        roi_imgsz: int = 960,
    ) -> List[Detection]:
        """
        İki aşamalı tespit:
        1. predicted_roi verilmişse, önce ROI içinde tespit yap
        2. ROI içinde sonuç bulunamazsa, tüm görüntüyü tara
        """
        self._roi_verify_counter += 1
        frame_h, frame_w = frame.shape[:2]

        if predicted_roi is not None:
            roi = self._normalize_roi(predicted_roi, frame_w=frame_w, frame_h=frame_h)
            roi_dets = self.detect(frame, roi=roi, imgsz_override=roi_imgsz)
            if roi_dets:
                self._log.debug("DETECT", f"ROI hit: {len(roi_dets)} detections")
                best_roi = roi_dets[0]
                suspicious_roi = self._is_suspicious_detection(best_roi, frame_w, frame_h)
                periodic_verify = (self._roi_verify_counter % 8) == 0
                if not suspicious_roi and not periodic_verify:
                    return roi_dets

                full_dets = self.detect(frame)
                if not full_dets:
                    return roi_dets

                best_full = full_dets[0]
                chosen = self._choose_detection(best_roi, best_full, frame_w, frame_h)
                if chosen is best_full:
                    self._log.info(
                        "DETECT",
                        "ROI verify: full-frame detection selected over ROI candidate",
                        debounce=0.2,
                    )
                    return full_dets
                return roi_dets

        return self.detect(frame)

    def _normalize_roi(
        self,
        roi: Tuple[int, int, int, int],
        frame_w: int,
        frame_h: int,
    ) -> Tuple[int, int, int, int]:
        """Tahmini ROI'yi minimum arama penceresine genişlet ve görüntü içine kırp."""
        x1, y1, x2, y2 = [int(v) for v in roi]
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0

        width = max(x2 - x1, int(frame_w * 0.22))
        height = max(y2 - y1, int(frame_h * 0.22))

        nx1 = int(round(cx - width / 2.0))
        ny1 = int(round(cy - height / 2.0))
        nx2 = int(round(cx + width / 2.0))
        ny2 = int(round(cy + height / 2.0))

        nx1 = max(0, nx1)
        ny1 = max(0, ny1)
        nx2 = min(frame_w, nx2)
        ny2 = min(frame_h, ny2)
        return nx1, ny1, nx2, ny2

    @staticmethod
    def _bbox_touches_edge(
        bbox: np.ndarray,
        frame_w: int,
        frame_h: int,
        margin_ratio: float = 0.02,
    ) -> bool:
        margin_x = frame_w * margin_ratio
        margin_y = frame_h * margin_ratio
        return (
            bbox[0] <= margin_x
            or bbox[1] <= margin_y
            or bbox[2] >= frame_w - margin_x
            or bbox[3] >= frame_h - margin_y
        )

    def _is_suspicious_detection(self, det: Detection, frame_w: int, frame_h: int) -> bool:
        width = float(det.bbox[2] - det.bbox[0])
        height = float(det.bbox[3] - det.bbox[1])
        area_ratio = max(width * height, 1.0) / float(frame_w * frame_h)
        tiny = area_ratio < 0.00035
        weak = det.confidence < max(self._conf + 0.05, 0.22)
        edge = self._bbox_touches_edge(det.bbox, frame_w, frame_h)
        return tiny or weak or edge

    def _choose_detection(
        self,
        roi_det: Detection,
        full_det: Detection,
        frame_w: int,
        frame_h: int,
    ) -> Detection:
        """ROI ve tam-kare adayı arasından daha güvenilir olanı seç."""
        roi_area = max(float((roi_det.bbox[2] - roi_det.bbox[0]) * (roi_det.bbox[3] - roi_det.bbox[1])), 1.0)
        full_area = max(float((full_det.bbox[2] - full_det.bbox[0]) * (full_det.bbox[3] - full_det.bbox[1])), 1.0)
        roi_edge = self._bbox_touches_edge(roi_det.bbox, frame_w, frame_h)
        full_edge = self._bbox_touches_edge(full_det.bbox, frame_w, frame_h)
        center_gap = np.hypot(
            roi_det.centroid[0] - full_det.centroid[0],
            roi_det.centroid[1] - full_det.centroid[1],
        )
        same_target = center_gap <= max(160.0, np.sqrt(roi_area) * 5.0)

        if same_target:
            if (full_area >= roi_area * 1.20 and full_det.confidence >= roi_det.confidence - 0.10) or (not full_edge and roi_edge):
                return full_det
            return roi_det

        if roi_edge and not full_edge and full_det.confidence >= roi_det.confidence:
            return full_det
        return roi_det
