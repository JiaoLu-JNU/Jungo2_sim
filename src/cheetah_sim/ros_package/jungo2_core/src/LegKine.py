#!/usr/bin/env python3
import numpy as np
from math import sqrt, sin, cos


class Leg(object):

    def __init__(self, d, a1, a2):
        self._d = d
        self._a1 = a1
        self._a2 = a2

    def jacobian(self, angle):
        t1 = angle[0]
        t2 = angle[1]
        t3 = angle[2]

        J11 = -self._a2 * sin(t1) * cos(t2 + t3) - self._a1 * \
            cos(t2) * sin(t1) - self._d * cos(t1)
        J12 = -self._a1 * cos(t1) * sin(t2) - self._a2 * cos(t1) * sin(t2 + t3)
        J13 = -self._a2 * cos(t1) * sin(t2 + t3)
        J21 = +self._a1 * cos(t1) * cos(t2) + self._a2 * \
            cos(t1) * cos(t2 + t3) - self._d * sin(t1)
        J22 = -self._a1 * sin(t1) * sin(t2) - self._a2 * sin(t1) * sin(t2 + t3)
        J23 = -self._a2 * sin(t1) * sin(t2 + t3)
        J31 = 0
        J32 = -self._a2 * cos(t2 + t3) - self._a1 * cos(t2)
        J33 = -self._a2 * cos(t2 + t3)

        return np.matrix([[J11, J12, J13],
                          [J21, J22, J23],
                          [J31, J32, J33]])

    def calc_end_pos(self, angle):
        t1 = angle[0]
        t2 = angle[1]
        t3 = angle[2]

        px = self._a1*cos(t1)*cos(t2) - self._d*sin(t1) + \
            self._a2*cos(t1)*cos(t2+t3)
        py = self._d*cos(t1) + self._a1*cos(t2)*sin(t1) + \
            self._a2*sin(t1)*cos(t2+t3)
        pz = -self._a1 * sin(t2) - self._a2 * sin(t2 + t3)

        return np.array([px, py, pz])

    def calc_joint_angle(self, pos):
        px = pos[0]
        py = pos[1]
        pz = pos[2]

        rho = sqrt(px ** 2 + py ** 2)
        t1 = np.arctan2(py, px) - np.arctan2(self._d /
                                             rho, sqrt(1 - (self._d/rho) ** 2))

        C3 = (px ** 2 + py ** 2 + pz ** 2 - self._a1 ** 2 -
              self._a2 ** 2 - self._d ** 2) / (2 * self._a1 * self._a2)
        S3 = -sqrt(1 - C3 ** 2)
        t3 = np.arctan2(S3, C3)

        rho = sqrt((self._a1 + self._a2 * C3) ** 2 + (self._a2 * S3) ** 2)
        t2 = np.arctan2(self._a1 + self._a2 * C3, self._a2 * S3) - \
            np.arctan2(sqrt(1 - (pz / rho) ** 2), -pz / rho)

        return np.array([t1, t2, t3])

    def calc_end_spd(self, angle, omega):
        omega = np.matrix(omega).transpose()
        J = self.jacobian(angle)
        return J * omega

    def calc_joint_spd(self, angle, speed):
        speed = np.matrix(speed).transpose()
        J = self.jacobian(angle)
        return J.I * speed

    def calc_end_force(self, angle, torque):
        torque = np.matrix(torque).transpose()
        J = self.jacobian(angle)
        return np.transpose(J).I * torque

    def calc_joint_torque(self, angle, force):
        force = np.matrix(force).transpose()
        J = self.jacobian(angle)
        return np.transpose(J) * force
