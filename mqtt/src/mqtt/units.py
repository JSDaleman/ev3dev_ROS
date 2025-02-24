#!/usr/bin/env python3

import math

class UnitConverter:
    @staticmethod
    def rpm_to_rad_per_sec(rpm: float) -> float:
        """Convierte revoluciones por minuto (RPM) a radianes por segundo (rad/s)."""
        return round(rpm * (math.pi / 30), 3)

    @staticmethod
    def rad_per_sec_to_rpm(rad_per_sec: float) -> float:
        """Convierte radianes por segundo (rad/s) a revoluciones por minuto (RPM)."""
        return round(rad_per_sec * (30 / math.pi), 3)


