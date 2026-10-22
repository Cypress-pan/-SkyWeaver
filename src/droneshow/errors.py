"""Shared service exceptions that must not import PyBullet."""
from __future__ import annotations


class RuntimeBusy(RuntimeError):
    """Raised when the runtime cannot service a request right now."""
