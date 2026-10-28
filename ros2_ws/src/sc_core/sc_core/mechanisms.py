"""Ground-truth 1-DOF mechanisms (paper Sec. 3.2), used by the offline
simulator and the evaluation. Never given to the controller or estimator."""

import numpy as np


class Prismatic:
    """Drawer: p(q) = p0 + q d*, constant axis."""

    def __init__(self, p0, axis, q_max=0.35):
        self.p0 = np.asarray(p0, dtype=float)
        self.axis = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
        self.q_max = q_max

    def position(self, q):
        return self.p0 + q * self.axis

    def tangent(self, q):
        return self.axis.copy()

    def opening(self, q):
        """Opening distance travelled by the handle [m]."""
        return q


class Revolute:
    """Door seen from above: hinge axis vertical, handle at radius rho.

    q is the opening angle; the handle starts at angle theta0 about the hinge.
    """

    def __init__(self, hinge, radius, theta0=0.0, q_max=np.deg2rad(90.0)):
        self.hinge = np.asarray(hinge, dtype=float)
        self.radius = radius
        self.theta0 = theta0
        self.q_max = q_max

    def position(self, q):
        th = self.theta0 + q
        return self.hinge + self.radius * np.array([np.cos(th), np.sin(th), 0.0])

    def tangent(self, q):
        th = self.theta0 + q
        return np.array([-np.sin(th), np.cos(th), 0.0])

    def opening(self, q):
        return self.radius * q
