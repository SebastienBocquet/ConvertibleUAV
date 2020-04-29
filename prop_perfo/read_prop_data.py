import glob
import numpy as np
from pathlib import Path
import pandas as pd
from dataclasses import dataclass
from typing import List
from pytest import approx
import matplotlib.pyplot as plt




@dataclass
class Propeller:
    '''Class representing the aerodynamic characteristics of a propeller'''
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
        return ct * rho * rpm**2 * self.diameter**4 / 3600

    def compute_torque(self, rpm, rho):
        cp = np.interp(rpm, self.cp[0], self.cp[1])
        p_aero = cp * rho * rpm**3 * self.diameter**5 / 60**3
        return p_aero / (2*np.pi/60 * rpm)


@dataclass
class Propulsion:
    '''Class for storing attributes of a propulsion set (battery, ESC, propeller and motor)'''
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
        return ct * rho * kv**2 * u0**2 / (3600 * 2000**2) * self.prop.diameter**4

    def compute_k1(self, rpm):
        kt = self.compute_kt(rpm)
        throttle = self.compute_throttle(rpm)
        return 2 * throttle * kt

    def compute_kq(self, rpm):
        ct = self.prop.compute_ct(rpm)
        cp = self.prop.compute_cp(rpm)
        return (1. / (2 * np.pi)) * (cp / ct) * self.prop.diameter


@dataclass
class TricopterAirframe:
    '''Class for storing attributes of a tricopter airframe characteristics
    (geometry, mass)'''
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
        t_a = self.mass * g / (2 * (1 + self.r_a * self.cos_alpha / self.r_b))
        t_b = 2 * self.r_a * self.cos_alpha * t_a / r_b
        return t_a, t_b


DATA_DIR = ['../../UIUC-propDB/volume-1/data',
            '../../UIUC-propDB/volume-2/data']

g = 9.81
rho = 1.205
diam_inch = 10
pitch_inch = 4.7
diam = diam_inch * 2.54 * 0.01
mass = 1.5
r_a = 0.425
r_b = 0.443
cos_alpha = 0.6647579365354364
sin_alpha = np.sin(np.arccos(cos_alpha))
kv = 1100
u0 = 10.8
# t_eq_a = mass * 9.81 / (2 * (1 + r_a * cos_alpha / r_b))
# t_eq_b = (2 * r_a * cos_alpha / r_b) * t_eq_a


props = []

for d in DATA_DIR:
    p = Path(d)
    filenames = list(p.glob('*%sx%s*static*.txt' %(str(diam_inch),
                                                   str(pitch_inch))))
    for f in filenames:
        data = pd.read_csv(f, delim_whitespace=True)
        props.append(Propeller(f.name,
                               diam,
                               np.array([data['RPM'], data['CT']]),
                               np.array([data['RPM'], data['CP']])))


def find_rpm(thrust, rho, propeller):
    rpm_next = 0
    rpm = 5000.
    error = np.inf
    nb_iter = 0
    while abs(error) > 1e-2 and nb_iter < 20:
        ct = propeller.compute_ct(rpm)
        rpm_next = 60 * (thrust / (ct * rho * propeller.diameter**4))**0.5
        error = rpm - rpm_next
        rpm = rpm_next
        nb_iter += 1
    assert nb_iter < 20
    # print('converged')
    return rpm_next


tri = TricopterAirframe(mass, r_a, r_b, cos_alpha)
t_eq_a, t_eq_b = tri.compute_thrust_equilibrium()
assert(2 * t_eq_a + t_eq_b == mass * g)
print('eq thrust', t_eq_a, t_eq_b)

total_thrusts = np.linspace(2., 30., 10)

for prop in props:
    print('prop', prop.name)
    th_a_to_0 = list()
    th_b_to_0 = list()
    for total_thrust in total_thrusts:
        propu = Propulsion(kv, u0, prop)
        t_0 = total_thrust / 3
        rpm_0 = find_rpm(t_0, rho, prop)
        th_0 = propu.compute_throttle(rpm_0)
        kt = propu.compute_kt(rpm_0)
        assert t_0 == approx(kt*th_0**2, rel=1e-3)
        t_a = total_thrust / (2 * (1 + tri.r_a * tri.cos_alpha / tri.r_b))
        rpm_a = find_rpm(t_a, rho, prop)
        t_b = 2 * tri.r_a * tri.cos_alpha * t_a / r_b
        total_thrust = 2 * t_a + t_b
        rpm_b = find_rpm(t_b, rho, prop)
        th_a = propu.compute_throttle(rpm_a)
        th_b = propu.compute_throttle(rpm_b)
        print(total_thrust, th_a, th_b)
        th_a_to_0.append(th_a / th_0)
        th_b_to_0.append(th_b / th_0)

    plt.plot(total_thrusts, th_a_to_0, label=f"th_a/th_0 {prop.name}")
    plt.plot(total_thrusts, th_b_to_0, label=f"th_b/th_0 {prop.name}")

plt.legend()
plt.savefig(f"th_ratios_database.png")
plt.close()

for prop in props:
    print('prop', prop.name)
    th_a_to_0 = list()
    th_b_to_0 = list()
    for total_thrust in total_thrusts:
        propu = Propulsion(kv, u0, prop)
        t_0 = total_thrust / 3
        rpm_0 = find_rpm(t_0, rho, prop)
        th_0 = propu.compute_throttle(rpm_0)
        kt = propu.compute_kt(rpm_0)
        assert t_0 == approx(kt*th_0**2, rel=1e-3)

        t_a = total_thrust / (2 * (1 + tri.r_a * tri.cos_alpha / tri.r_b))
        t_b = 2 * tri.r_a * tri.cos_alpha * t_a / r_b

        th_a = th_0 + (t_a**0.5 - t_0**0.5) / kt**0.5
        th_b = th_0 + (t_b**0.5 - t_0**0.5) / kt**0.5
        print(th_a, th_b)
        th_a_to_0.append(th_a / th_0)
        th_b_to_0.append(th_b / th_0)

    plt.plot(total_thrusts, th_a_to_0, label=f"th_a/th_0 {prop.name}")
    plt.plot(total_thrusts, th_b_to_0, label=f"th_b/th_0 {prop.name}")

plt.legend()
plt.savefig(f"th_ratios_theo.png")
plt.close()

# propu = Propulsion(kv, u0, prop)
# kt = propu.compute_kt(rpm)
# k1 = propu.compute_k1(rpm)
# kq = propu.compute_kq(rpm)
# beta_eq = kq * cos_alpha / (r_b * sin_alpha)
# yaw_control = 1000
# beta = (-2 * kq * k1 / (t_eq_a * r_a * sin_alpha)) * yaw_control
# print(beta_eq*180./np.pi, beta*180./np.pi)
# print(kt, k1, kq)
