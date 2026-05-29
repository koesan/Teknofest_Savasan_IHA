from __future__ import annotations

from dataclasses import dataclass
from typing import Optional
import math
import logging
import cv2
import numpy as np
from pyzbar.pyzbar import decode, ZBarSymbol


@dataclass
class QRResult:
    detected: bool = False
    decoded_text: str = ""
    polygon: Optional[np.ndarray] = None
    bbox: Optional[np.ndarray] = None
    in_target_area: bool = False
    meets_size_requirement: bool = False
    processed_frame: Optional[np.ndarray] = None
    raw_frame: Optional[np.ndarray] = None


class QRDetector:
    def __init__(self, frame_width: int, frame_height: int, target_area_margin: float):
        self.logger = logging.getLogger("QR")
        self.logger.info("PyZBar QR dedektörü başlatılıyor.")
        self._frame_width = frame_width
        self._frame_height = frame_height
        mx = int(frame_width * target_area_margin)
        my = int(frame_height * target_area_margin)
        self._target_area = (mx, my, frame_width - mx, frame_height - my)

    @property
    def target_area(self) -> tuple[int, int, int, int]:
        return self._target_area

    def detect(self, frame: np.ndarray) -> QRResult:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame

        # 1. Köşe tespiti yapılmış bölgeleri (ROI) ara
        rois = self._find_finder_pattern_rois(gray)
        
        best_detected = QRResult()

        for rx, ry, rw, rh in rois:
            crop = gray[ry : ry + rh, rx : rx + rw]
            res = self._decode_crop(crop, (rx, ry), 1.0, verified=True)
            if res.decoded_text:
                res.raw_frame = frame.copy()
                return res
            if res.detected and not best_detected.decoded_text:
                best_detected = res

        # 2. Köşe bulunamazsa resmi küçültüp genel tarama yap (hızlı tarama)
        if not best_detected.detected:
            h, w = gray.shape[:2]
            scale = 640.0 / w
            if scale < 1.0:
                gray_small = cv2.resize(gray, (640, int(h * scale)), interpolation=cv2.INTER_AREA)
                res = self._decode_crop(gray_small, (0, 0), scale, verified=False)
            else:
                res = self._decode_crop(gray, (0, 0), 1.0, verified=False)
                
            if res.decoded_text:
                res.raw_frame = frame.copy()
                return res
            if res.detected:
                best_detected = res

        if best_detected.detected:
            best_detected.raw_frame = frame.copy()

        return best_detected

    def _find_finder_pattern_rois(self, gray: np.ndarray) -> list[tuple[int, int, int, int]]:
        # Hızlı kontur analizi için resmi küçült
        h, w = gray.shape[:2]
        scale = 640.0 / w
        if scale < 1.0:
            gray_small = cv2.resize(gray, (640, int(h * scale)), interpolation=cv2.INTER_AREA)
        else:
            gray_small = gray
            scale = 1.0

        # Işık değişimlerine karşı adaptif eşikleme uygula
        thresh = cv2.adaptiveThreshold(
            gray_small, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 21, 2
        )
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if hierarchy is None:
            return []

        hierarchy = hierarchy[0]
        num_contours = len(contours)

        finder_candidates = []
        for i in range(num_contours):
            child = hierarchy[i][2]
            if child != -1:
                grandchild = hierarchy[child][2]
                if grandchild != -1:
                    x, y, cw, ch = cv2.boundingRect(contours[i])
                    aspect_ratio = float(cw) / ch
                    if 0.65 <= aspect_ratio <= 1.35 and cw >= 3 and ch >= 3:
                        if cw < gray_small.shape[1] * 0.5 and ch < gray_small.shape[0] * 0.5:
                            finder_candidates.append((x, y, cw, ch))

        if not finder_candidates:
            return []

        # Yakın köşeleri gruplandır
        groups = []
        used = set()
        for idx, (x, y, cw, ch) in enumerate(finder_candidates):
            if idx in used:
                continue
            group = [finder_candidates[idx]]
            used.add(idx)
            max_dist = max(cw, ch) * 12.0
            for jdx, (xj, yj, wj, hj) in enumerate(finder_candidates):
                if jdx in used:
                    continue
                c1 = (x + cw / 2.0, y + ch / 2.0)
                c2 = (xj + wj / 2.0, yj + hj / 2.0)
                dist = math.sqrt((c1[0] - c2[0]) ** 2 + (c1[1] - c2[1]) ** 2)
                if dist < max_dist and abs(cw - wj) / max(cw, wj) < 0.6:
                    group.append(finder_candidates[jdx])
                    used.add(jdx)
            groups.append(group)

        rois = []
        for group in groups:
            xs = [item[0] for item in group]
            ys = [item[1] for item in group]
            ws = [item[2] for item in group]
            hs = [item[3] for item in group]

            min_x = min(xs)
            min_y = min(ys)
            max_x = max([x + cw for x, cw in zip(xs, ws)])
            max_y = max([y + ch for y, ch in zip(ys, hs)])

            if len(group) == 1:
                # Tek köşe varsa etrafını genişlet
                w_fp = ws[0]
                h_fp = hs[0]
                min_x = int(min_x - 3.5 * w_fp)
                min_y = int(min_y - 3.5 * h_fp)
                max_x = int(max_x + 3.5 * w_fp)
                max_y = int(max_y + 3.5 * h_fp)
            else:
                # Kenar boşluğu ekle
                pad_w = int((max_x - min_x) * 0.3)
                pad_h = int((max_y - min_y) * 0.3)
                min_x -= pad_w
                min_y -= pad_h
                max_x += pad_w
                max_y += pad_h

            # Koordinatları orijinal boyutlara ölçekle
            min_x = int(min_x / scale)
            min_y = int(min_y / scale)
            max_x = int(max_x / scale)
            max_y = int(max_y / scale)

            # Sınırları kırp
            min_x = max(0, min_x)
            min_y = max(0, min_y)
            max_x = min(gray.shape[1], max_x)
            max_y = min(gray.shape[0], max_y)

            if max_x - min_x >= 16 and max_y - min_y >= 16:
                rois.append((min_x, min_y, max_x - min_x, max_y - min_y))

        return rois

    def _decode_crop(
        self, gray_crop: np.ndarray, offset: tuple[int, int], scale: float, verified: bool
    ) -> QRResult:
        try:
            # Çok küçükse PyZBar'ın okuyabilmesi için büyüt
            h, w = gray_crop.shape[:2]
            target_w = 400
            if w < target_w:
                s_resize = target_w / w
                resized = cv2.resize(gray_crop, (target_w, int(h * s_resize)), interpolation=cv2.INTER_CUBIC)
                total_scale = scale * s_resize
            else:
                resized = gray_crop
                total_scale = scale

            # PyZBar ile çözmeyi dene
            variants = [resized]
            if verified:
                # Başarı oranını artırmak için keskinleştirilmiş ve siyah-beyaz hallerini de dene
                variants.append(self._sharpen(resized))
                variants.append(cv2.threshold(resized, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1])

            for img in variants:
                decoded_objs = decode(img, symbols=[ZBarSymbol.QRCODE])
                for obj in decoded_objs:
                    text = (obj.data.decode("utf-8") or "").strip()
                    if obj.polygon and len(obj.polygon) >= 4:
                        pts = np.array([(p.x, p.y) for p in obj.polygon], dtype=float)
                        # Koordinatları orijinal çerçeveye geri eşle
                        pts = pts / (total_scale / scale) / scale + np.array(offset, dtype=float)
                        result = self._build_result(pts, text)
                        if text:
                            # Başarılı deşifre görselini işaretle ve kaydet
                            annotated_crop = img.copy()
                            if annotated_crop.ndim == 2:
                                annotated_crop = cv2.cvtColor(annotated_crop, cv2.COLOR_GRAY2BGR)
                            crop_pts = np.array([(p.x, p.y) for p in obj.polygon], dtype=np.int32).reshape(-1, 1, 2)
                            cv2.polylines(annotated_crop, [crop_pts], isClosed=True, color=(0, 255, 0), thickness=3)
                            result.processed_frame = annotated_crop
                            return result

            if verified:
                # Okunamadıysa bile tespit edildi olarak işaretle
                pts = np.array([
                    [offset[0], offset[1]],
                    [offset[0] + w, offset[1]],
                    [offset[0] + w, offset[1] + h],
                    [offset[0], offset[1] + h]
                ], dtype=float)
                return self._build_result(pts, "")
        except Exception:
            pass
        return QRResult()

    def _sharpen(self, img: np.ndarray) -> np.ndarray:
        kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]], dtype=np.float32)
        return cv2.filter2D(img, -1, kernel)

    def _build_result(self, points: np.ndarray, decoded_text: str) -> QRResult:
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        if pts.shape[0] < 4:
            return QRResult()

        x1 = float(np.min(pts[:, 0]))
        y1 = float(np.min(pts[:, 1]))
        x2 = float(np.max(pts[:, 0]))
        y2 = float(np.max(pts[:, 1]))
        ax1, ay1, ax2, ay2 = self._target_area
        in_target = bool(
            np.all(pts[:, 0] >= ax1)
            and np.all(pts[:, 0] <= ax2)
            and np.all(pts[:, 1] >= ay1)
            and np.all(pts[:, 1] <= ay2)
        )
        width_ratio = (x2 - x1) / self._frame_width
        height_ratio = (y2 - y1) / self._frame_height
        meets_size = bool(width_ratio >= 0.05 or height_ratio >= 0.05)
        return QRResult(
            detected=True,
            decoded_text=decoded_text,
            polygon=pts,
            bbox=np.array([x1, y1, x2, y2], dtype=float),
            in_target_area=in_target,
            meets_size_requirement=meets_size,
            processed_frame=None,
        )
