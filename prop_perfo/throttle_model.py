import numpy as np
import pandas as pd
from pathlib import Path
from pytest import approx
import matplotlib.pyplot as plt
from propulsion_model import Propeller, Propulsion, TricopterAirframe


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
        rpm_0 = prop.find_rpm(t_0, rho)
        th_0 = propu.compute_throttle(rpm_0)
        kt = propu.compute_kt(rpm_0)
        assert t_0 == approx(kt*th_0**2, rel=1e-3)
        t_a = total_thrust / (2 * (1 + tri.r_a * tri.cos_alpha / tri.r_b))
        rpm_a = prop.find_rpm(t_a, rho)
        t_b = 2 * tri.r_a * tri.cos_alpha * t_a / r_b
        total_thrust = 2 * t_a + t_b
        rpm_b = prop.find_rpm(t_b, rho)
        th_a = propu.compute_throttle(rpm_a)
        th_b = propu.compute_throttle(rpm_b)
        print(total_thrust, th_a, th_b)
        th_a_to_0.append(th_a / th_0)
        th_b_to_0.append(th_b / th_0)

    plt.plot(total_thrusts, th_a_to_0, label=f"th_a/th_0 {prop.name}")
    plt.plot(total_thrusts, th_b_to_0, label=f"th_b/th_0 {prop.name}")

plt.legend()
plt.xlabel('total thrust (N)')
plt.savefig(f"th_ratios_database.png")
plt.close()

for prop in props:
    print('prop', prop.name)
    th_a_to_0 = list()
    th_b_to_0 = list()
    for total_thrust in total_thrusts:
        propu = Propulsion(kv, u0, prop)
        t_0 = total_thrust / 3
        rpm_0 = prop.find_rpm(t_0, rho)
        th_0 = propu.compute_throttle(rpm_0)
        kt = propu.compute_kt(rpm_0)
        assert t_0 == approx(kt*th_0**2, rel=1e-3)

        t_a = total_thrust / (2 * (1 + tri.r_a * tri.cos_alpha / tri.r_b))
        t_b = 2 * tri.r_a * tri.cos_alpha * t_a / r_b

        th_a = th_0 + (t_a**0.5 - t_0**0.5) / kt**0.5
        th_b = th_0 + (t_b**0.5 - t_0**0.5) / kt**0.5
        print(th_a / th_0, th_b / th_0)
        th_a_to_0.append(th_a / th_0)
        th_b_to_0.append(th_b / th_0)

    plt.plot(total_thrusts, th_a_to_0, label=f"th_a/th_0 {prop.name}")
    plt.plot(total_thrusts, th_b_to_0, label=f"th_b/th_0 {prop.name}")

plt.legend()
plt.xlabel('total thrust (N)')
plt.savefig(f"th_ratios_theo.png")
plt.close()
