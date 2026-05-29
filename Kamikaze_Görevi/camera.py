from __future__ import annotations

import threading
import time
from typing import Optional

import cv2
import numpy as np

from logger import get_logger


class Camera:
    def __init__(self, source: str, name: str, width: int, height: int):
        self._log = get_logger()
        self._source = source
        self._name = name
        self._width = width
        self._height = height
        self._lock = threading.Lock()
        self._frame: Optional[np.ndarray] = None
        self._frame_time = 0.0
        self._cap = None
        self._subscriber = None
        self._bridge = None
        self._is_ros = source.startswith("/")
        self._shape_logged = False

        if self._is_ros:
            self._init_ros()
        else:
            self._init_cv()

    def _init_ros(self):
        try:
            import rospy
            from cv_bridge import CvBridge
            from sensor_msgs.msg import Image

            if not rospy.core.is_initialized():
                rospy.init_node("vtol_kamikaze_camera", anonymous=True, disable_signals=True)

            self._bridge = CvBridge()
            self._subscriber = rospy.Subscriber(self._source, Image, self._ros_callback, queue_size=1)
            try:
                msg = rospy.wait_for_message(self._source, Image, timeout=3.0)
                self._ros_callback(msg)
            except Exception:
                self._log.warn("CAMERA", f"waiting first frame: {self._source}")
            self._log.info("CAMERA", f"ros topic active: {self._source}")
        except Exception as exc:
            self._log.warn("CAMERA", f"ros init failed, fallback to OpenCV: {exc}")
            self._is_ros = False
            self._source = "0"
            self._init_cv()

    def _ros_callback(self, msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
            with self._lock:
                self._frame = frame
                self._frame_time = time.time()
            self._log_frame_shape(frame)
        except Exception as exc:
            self._log.warn("CAMERA", f"ros callback failed: {exc}", debounce=2.0)

    def _init_cv(self):
        try:
            source = int(self._source)
        except ValueError:
            source = self._source
        self._cap = cv2.VideoCapture(source)
        if self._cap.isOpened():
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
            self._log.info("CAMERA", f"opencv source active: {self._source}")
        else:
            self._log.error("CAMERA", f"camera open failed: {self._source}")

    def read(self) -> Optional[np.ndarray]:
        if self._is_ros:
            with self._lock:
                if self._frame is None:
                    return None
                return self._frame.copy()

        if self._cap is None or not self._cap.isOpened():
            return None

        ok, frame = self._cap.read()
        if not ok:
            return None
        with self._lock:
            self._frame = frame
            self._frame_time = time.time()
        self._log_frame_shape(frame)
        return frame

    def _log_frame_shape(self, frame: np.ndarray):
        if self._shape_logged or frame is None:
            return
        h, w = frame.shape[:2]
        self._log.info("CAMERA", f"frame size={w}x{h} expected={self._width}x{self._height}")
        if w != self._width or h != self._height:
            self._log.warn("CAMERA", "frame size differs from config; QR target area may be scaled wrong")
        self._shape_logged = True

    def close(self):
        if self._cap is not None:
            self._cap.release()
        if self._subscriber is not None:
            try:
                self._subscriber.unregister()
            except Exception:
                pass
