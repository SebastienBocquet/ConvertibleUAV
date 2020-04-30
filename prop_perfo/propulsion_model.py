import glob
from dataclasses import dataclass
from pathlib import Path
from typing import List

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from pytest import approx

rho = 1.205


@dataclass
class Propeller:
    """Class representing the aerodynamic characteristics of a propeller"""

    def __init__(self, name, diameter, ct, cp):
        self.name = name
        self.diameter = diameter
        self.ct = ct
        self.cp = cp

    namname: str
    diameter: float
    ct: np.ndarray
    cp: np.ndarray

    def compute_ct(self, rpm):
        return np.interp(rpm, self.ct[0], self.ct[1])

    def compute_cp(self, rpm):
        return np.interp(rpm, self.cp[0], self.cp[1])

    def compute_thrust(self, rpm, rho):
        ct = self.compute_ct(rpm)
        return ct * rho * rpm ** 2 * self.diameter ** 4 / 3600

    def compute_torque(self, rpm, rho):
        cp = np.interp(rpm, self.cp[0], self.cp[1])
        p_aero = cp * rho * rpm ** 3 * self.diameter ** 5 / 60 ** 3
        return p_aero / (2 * np.pi / 60 * rpm)

    def find_rpm(self, thrust, rho):
        rpm_next = 0
        rpm = 5000.0
        error = np.inf
        nb_iter = 0
        while abs(error) > 1e-2 and nb_iter < 20:
            ct = self.compute_ct(rpm)
            rpm_next = 60 * (thrust / (ct * rho * self.diameter ** 4)) ** 0.5
            error = rpm - rpm_next
            rpm = rpm_next
            nb_iter += 1
        assert nb_iter < 20
        return rpm_next


@dataclass
class Propulsion:
    """Class for storing attributes of a propulsion set (battery, ESC, propeller and motor)"""

    def __init__(self, kv, u0, propeller):
        self.kv = kv
        self.u0 = u0
        self.prop = propeller

    kv: int
    u0: float
    prop: Propeller

    def compute_throttle(self, rpm):
        return rpm / (self.kv * self.u0 / 2000)

    def compute_kt(self, rpm):
        ct = self.prop.compute_ct(rpm)
        return (
            ct
            * rho
            * self.kv ** 2
            * self.u0 ** 2
            / (3600 * 2000 ** 2)
            * self.prop.diameter ** 4
        )

    def compute_k1(self, rpm):
        kt = self.compute_kt(rpm)
        throttle = self.compute_throttle(rpm)
        return 2 * throttle * kt

    def compute_kq(self, rpm):
        ct = self.prop.compute_ct(rpm)
        cp = self.prop.compute_cp(rpm)
        return (1.0 / (2 * np.pi)) * (cp / ct) * self.prop.diameter


@dataclass
class TricopterAirframe:
    """Class for storing attributes of a tricopter airframe characteristics
    (geometry, mass)"""

    def __init__(self, mass, r_a, r_b, cos_alpha):
        self.mass = mass
        self.r_a = r_a
        self.r_b = r_b
        self.cos_alpha = cos_alpha

    mass: float
    r_a: float
    r_b: float
    cos_alpha: float

    def compute_thrust_equilibrium(self):
        t_a = self.mass * 9.81 / (2 * (1 + self.r_a * self.cos_alpha / self.r_b))
        t_b = 2 * self.r_a * self.cos_alpha * t_a / self.r_b
        return t_a, t_b


