#!/usr/bin/env python3
import numpy as np
import cvxpy as cp
import time


class Grfs(object):

    def __init__(self):
        self._mu = 0.8
        self._f_min = 2
        self._f_max = 100
        self._S = np.diag([1, 1, 0.2, 20, 20, 5])
        self._W = np.diag([0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001,
                           0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001])

    @property
    def S(self):
        return self._S

    @S.setter
    def S(self, S):
        self._S = np.diag(S)

    def _get_mat_A_subblock(self, p_com):
        dx = p_com[0]
        dy = p_com[1]
        dz = p_com[2]
        return np.array([[0, -dz, dy],
                         [dz, 0, -dx],
                         [-dy, dx, 0]])

    def _get_mat_C_subblock(self, mu):
        return np.array([[1, 0, -mu],
                         [0, 1, -mu],
                         [0, 1, mu],
                         [1, 0, mu],
                         [0, 0, 1]])

    def _get_mat_A(self, p_com1, p_com2, p_com3, p_com4):
        I = np.eye(3)
        B1 = self._get_mat_A_subblock(p_com1)
        B2 = self._get_mat_A_subblock(p_com2)
        B3 = self._get_mat_A_subblock(p_com3)
        B4 = self._get_mat_A_subblock(p_com4)
        return np.block([[I, I, I, I],
                         [B1, B2, B3, B4]])

    def _get_mat_f(self):
        f = cp.Variable(12)
        return f

    def _get_mat_b(self, f_com, t_com):
        b = np.block([f_com, t_com])
        return b

    def _get_mat_C(self, mu):
        N = np.zeros((5, 3))
        B = self._get_mat_C_subblock(mu)
        return np.block([[B, N, N, N],
                         [N, B, N, N],
                         [N, N, B, N],
                         [N, N, N, B]])

    def _get_mat_D_min(self, f_min):
        D = np.array([-float("inf"), -float("inf"), 0, 0, f_min])
        return np.block([D, D, D, D])

    def _get_mat_D_max(self, f_max):
        D = np.array([0, 0, float("inf"), float("inf"), f_max])
        return np.block([D, D, D, D])

    def solve(self, p_com1, p_com2, p_com3, p_com4, f_com, t_com):
        A = self._get_mat_A(p_com1, p_com2, p_com3, p_com4)
        b = self._get_mat_b(f_com, t_com)
        C = self._get_mat_C(0.8)
        D1 = self._get_mat_D_min(2)
        D2 = self._get_mat_D_max(100)
        x = self._get_mat_f()
        objective = cp.Minimize(cp.quad_form(A * x - b, self._S)
                                + cp.quad_form(x, self._W))
        constraints = [D1 <= C*x, C*x <= D2]
        prob = cp.Problem(objective, constraints)
        prob.solve()
        print(x.value)
        return x.value


if __name__ == '__main__':
    grfs = Grfs()
    p_com1 = np.array([0.18, 0.12, 0.3])
    p_com2 = np.array([0.18, -0.12, 0.2])
    p_com3 = np.array([-0.18, 0.12, 0.3])
    p_com4 = np.array([-0.18, -0.12, 0.3])
    f_com = np.array([10, 0, 98])
    t_com = np.array([0, 0, 0])
    t1 = time.time_ns()
    grfs.solve(p_com1, p_com2, p_com3, p_com4, f_com, t_com)
    t2 = time.time_ns()
    print(f"Time: {(t2-t1)/1000000}ms")

    p_com1 = np.array([0.18, 0.12, 0.3])
    p_com2 = np.array([0.18, -0.12, 0.3])
    p_com3 = np.array([-0.18, 0.12, 0.3])
    p_com4 = np.array([-0.18, -0.12, 0.3])
    f_com = np.array([0, 10, 98])
    t_com = np.array([0, 0, 0])
    t1 = time.time_ns()
    grfs.solve(p_com1, p_com2, p_com3, p_com4, f_com, t_com)
    t2 = time.time_ns()
    print(f"Time: {(t2-t1)/1000000}ms")

    p_com1 = np.array([0.18, 0.12, 0.1])
    p_com2 = np.array([0.18, -0.12, 0.1])
    p_com3 = np.array([-0.18, 0.12, 0.3])
    p_com4 = np.array([-0.18, -0.12, 0.3])
    f_com = np.array([10, 10, 98])
    t_com = np.array([0, 0, 0])
    t1 = time.time_ns()
    grfs.solve(p_com1, p_com2, p_com3, p_com4, f_com, t_com)
    t2 = time.time_ns()
    print(f"Time: {(t2-t1)/1000000}ms")
