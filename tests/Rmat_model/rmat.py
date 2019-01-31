"""
reconstruct the rotation matrix of an aircraft
"""

import numpy as np

def compute_rmat(psi, theta, phi):

    rxx = np.cos(theta) * np.cos(psi)
    rxy = np.sin(phi) * np.sin(theta) * np.cos(psi) - np.cos(phi) * np.sin(psi)
    rxz = np.cos(phi) * np.sin(theta) * np.cos(psi) + np.sin(phi) * np.sin(psi)

    ryx = np.cos(theta) * np.sin(psi)
    ryy = np.sin(phi) * np.sin(theta) * np.sin(psi) + np.cos(phi) * np.cos(psi)
    ryz = np.cos(phi) * np.sin(theta) * np.sin(psi) - np.sin(phi) * np.cos(psi)

    rzx = - np.sin(theta)
    rzy = np.sin(phi) * np.cos(theta)
    rzz = np.cos(phi) * np.cos(theta)

    rmat = np.zeros((9))

    rmat[0] = ryy
    rmat[1] = -ryx
    rmat[2] = -ryz
    rmat[3] = -rxy
    rmat[4] = rxx
    rmat[5] = rxz
    rmat[6] = -rzy
    rmat[7] = rzx
    rmat[8] = rzz   

    return rmat

def print_rmat(rmat):
    print("[%.2f %.2f %.2f" %(rmat[0], rmat[1], rmat[2]))
    print(" %.2f %.2f %.2f" %(rmat[3], rmat[4], rmat[5]))
    print(" %.2f %.2f %.2f]" %(rmat[6], rmat[7], rmat[8]))

print_rmat(compute_rmat(0., 0., 0.))
