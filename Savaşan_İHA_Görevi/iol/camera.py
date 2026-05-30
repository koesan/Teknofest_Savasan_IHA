"""
Savaşan İHA - Camera Interface

ROS subscriber veya OpenCV VideoCapture tabanlı kamera arayüzü.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import cv2
import numpy as np

from core.logger import get_logger


class Camera:
    """
    Thread-safe kamera arayüzü.
    ROS topic veya video dosyası/kamera girişi destekler.
    """

    def __init__(
        self,
        source: str,
        name: str = "camera",
        width: int = 1280,
        height: int = 720,
    ):
        self._log = get_logger()
        self._name = name
        self._source = source
        self._width = width
        self._height = height

        self._frame: Optional[np.ndarray] = None
        self._frame_time: float = 0.0
        self._lock = threading.Lock()
        self._running = False
        self._is_ros = source.startswith("/")

        if self._is_ros:
            self._init_ros(source)
        else:
            self._init_opencv(source)

    def _init_ros(self, topic: str):
        """ROS subscriber başlat."""
        try:
            import rospy
            from sensor_msgs.msg import Image
            from cv_bridge import CvBridge

            self._bridge = CvBridge()
            self._subscriber = rospy.Subscriber(
                topic, Image, self._ros_callback, queue_size=1
            )
            self._running = True
            self._log.info("CAMERA",
                f"[{self._name}] ROS subscriber: {topic}")
        except ImportError:
            self._log.warn("CAMERA",
                f"[{self._name}] ROS not available, trying OpenCV fallback")
            self._is_ros = False
            self._init_opencv("0")

    def _ros_callback(self, msg):
        """ROS görüntü geri çağırma (callback) fonksiyonu."""
        try:
            from cv_bridge import CvBridge
            bridge = CvBridge()
            frame = bridge.imgmsg_to_cv2(msg, "bgr8")
            with self._lock:
                self._frame = frame
                self._frame_time = time.time()
        except Exception as e:
            self._log.error("CAMERA", f"[{self._name}] ROS callback error: {e}")

    def _init_opencv(self, source: str):
        """OpenCV VideoCapture başlat."""
        try:
            src = int(source)
        except ValueError:
            src = source

        self._cap = cv2.VideoCapture(src)
        if self._cap.isOpened():
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
            self._running = True
            self._log.info("CAMERA",
                f"[{self._name}] OpenCV capture: {source}")
        else:
            self._running = False
            self._log.error("CAMERA",
                f"[{self._name}] Failed to open: {source}")

    def read(self) -> Optional[np.ndarray]:
        """
        Son frame'i al.

        Dönen:
            BGR görüntü dizisi (frame) veya None
        """
        if self._is_ros:
            with self._lock:
                return self._frame.copy() if self._frame is not None else None

        if hasattr(self, '_cap') and self._cap is not None and self._cap.isOpened():
            ret, frame = self._cap.read()
            if ret:
                with self._lock:
                    self._frame = frame
                    self._frame_time = time.time()
                return frame
        return None

    @property
    def frame_age(self) -> float:
        """Son frame'in yaşı (saniye)."""
        if self._frame_time == 0:
            return float('inf')
        return time.time() - self._frame_time

    def close(self):
        """Kaynakları serbest bırak."""
        self._running = False
        if hasattr(self, '_cap') and self._cap is not None:
            self._cap.release()
        if self._is_ros:
            try:
                self._subscriber.unregister()
            except Exception:
                pass
        self._log.info("CAMERA", f"[{self._name}] Closed")
