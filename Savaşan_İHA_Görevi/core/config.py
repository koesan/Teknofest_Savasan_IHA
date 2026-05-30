"""
Savaşan İHA - YAML Config Loader
"""

from __future__ import annotations

import os
import copy
from typing import Any, Dict, Optional

import yaml


class Config:
    """Nokta (.) erişimli ve katmanlı yapılandırma desteği sunan YAML yükleyici."""

    def __init__(self, data: Dict[str, Any]):
        self._data = data

    # ---- fabrika metotları -------------------------------------------------
    @classmethod
    def from_yaml(cls, path: str, overlay_path: Optional[str] = None) -> "Config":
        """YAML dosyasını yükler. İsteğe bağlı olarak üzerine ek yapılandırma birleştirir."""
        with open(path, "r", encoding="utf-8") as f:
            base = yaml.safe_load(f)
        if overlay_path and os.path.exists(overlay_path):
            with open(overlay_path, "r", encoding="utf-8") as f:
                overlay = yaml.safe_load(f) or {}
            base = cls._deep_merge(base, overlay)
        return cls(base)

    # ---- erişim -----------------------------------------------------------
    def __getattr__(self, name: str) -> Any:
        if name.startswith("_"):
            return super().__getattribute__(name)
        try:
            val = self._data[name]
        except KeyError:
            raise AttributeError(f"Config has no key '{name}'")
        if isinstance(val, dict):
            return Config(val)
        return val

    def __getitem__(self, key: str) -> Any:
        return self.__getattr__(key)

    def get(self, key: str, default: Any = None) -> Any:
        try:
            return self.__getattr__(key)
        except AttributeError:
            return default

    def to_dict(self) -> Dict[str, Any]:
        return copy.deepcopy(self._data)

    def __repr__(self) -> str:
        return f"Config({self._data})"

    def __contains__(self, key: str) -> bool:
        return key in self._data

    # ---- yardımcılar ------------------------------------------------------
    @staticmethod
    def _deep_merge(base: dict, overlay: dict) -> dict:
        result = copy.deepcopy(base)
        for k, v in overlay.items():
            if k in result and isinstance(result[k], dict) and isinstance(v, dict):
                result[k] = Config._deep_merge(result[k], v)
            else:
                result[k] = copy.deepcopy(v)
        return result
