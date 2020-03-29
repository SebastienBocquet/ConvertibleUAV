import glob
import numpy as np
from pathlib import Path
import pandas as pd


DATA_DIR = ['../../UIUC-propDB/volume-1/data',
            '../../UIUC-propDB/volume-2/data']

rho = 1.205
diam_inch = 10
pitch_inch = 4.7
diam = diam_inch * 2.54 * 0.01
mass = 1.5
r_a = 0.425
r_b = 0.443
cos_alpha = 0.6647579365354364
kv = 1100
u0 = 10.8
t_eq_a = mass * 9.81 / (2 * (1 + r_a * cos_alpha / r_b))
t_eq_b = (2 * r_a * cos_alpha / r_b) * t_eq_a

print(diam)
print(t_eq_a, t_eq_b)


for d in DATA_DIR:
    p = Path(d)
    filenames = list(p.glob('*%sx%s*static*.txt' %(str(diam_inch),
                                                   str(pitch_inch))))
    for f in filenames:
        print(f)
        coef = {}
        data = pd.read_csv(f, delim_whitespace=True)
        ct_mean = np.mean(data['CT'])
        n_eq = 60 * (t_eq_b / (ct_mean * rho * diam**4))**0.5
        ct_eq = np.interp(n_eq, data['RPM'], data['CT']) 
        cp_eq = np.interp(n_eq, data['RPM'], data['CP']) 
        kt_eq = ct_eq * rho * kv**2 * u0**2 / (3600 * 2000**2) * diam**4
        throttle_eq = n_eq / (kv * u0 / 2000)
        print(throttle_eq)
        k1 = 2 * throttle_eq * kt_eq
        k_q_eq = (1. / (2 * np.pi)) * (cp_eq / ct_eq) * diam
        coef['kq'] = k_q_eq
        coef['kt'] = kt_eq
        coef['k1'] = k1
        print(coef)
        th = throttle_eq
        t = coef['kt'] * th**2
        q = coef['kq'] * t
        t1 = coef['kt'] * (throttle_eq**2 - (th - throttle_eq) * 2 *
                           throttle_eq)
        print(t,t1,q)
        p_aero = cp_eq * rho * (n_eq/60)**3 * diam**5
        q = p_aero / (2*np.pi/60 * n_eq)
        print(p_aero, q)
