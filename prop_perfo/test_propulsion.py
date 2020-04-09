import glob
import numpy as np
from pathlib import Path
import pandas as pd
from dataclasses import dataclass
from typing import List
from pytest import approx
from read_prop_data import Propeller, Propulsion


rho = 1.205
diam = 0.254
kv = 1100
u0 = 10.8


def test_thrust():
    rpm = 5600
    prop = Propeller('apc',
                     diam,
                     np.array([[rpm], [0.1]]), 
                     np.array([[rpm], [0.08]]))
    propulsion = Propulsion(kv, u0, prop)
    kt = propulsion.compute_kt(rpm)
    k1 = propulsion.compute_k1(rpm)
    throttle = propulsion.compute_throttle(rpm)
    thrust = kt * throttle**2
    thrust1 = k1 * throttle - kt * throttle**2
    approx(thrust, thrust1, abs=1e-9)
    approx(thrust, prop.compute_thrust(rpm, rho), abs=1e-9)
