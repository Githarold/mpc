import numpy as np
from scipy.linalg import expm

import cvxpy

class OptimizeCost:
    def __init__(self, MpcParams, ModelParams):
        self.MpcParams = MpcParams
        self.ModelParams = ModelParams

        self.N = MpcParams.N
        self.Ts = MpcParams.Ts
        self.R = MpcParams.R
        self.Rd = MpcParams.Rd
        self.Q = MpcParams.Q
        self.Qf = MpcParams.Qf

        self.sx = ModelParams.sx
        self.su = ModelParams.su
        self.MAX_STEER = ModelParams.MAX_STEER
        self.MAX_DSTEER = ModelParams.MAX_DSTEER
        self.MAX_SPEED = ModelParams.MAX_SPEED
        self.MIN_SPEED = ModelParams.MIN_SPEED
        self.MAX_WW = ModelParams.MAX_WW
        self.MIN_WW = ModelParams.MIN_WW
        
        self.opt_delta_pre = [0.0] * self.N
        self.opt_ww_pre = [1.0] * self.N

    def optimize(self, Xref, A_list, B_list, C_list, current_state): # *_list : 0 ~ N-1, Xref : 0 ~ N, currstate : state list
        x = cvxpy.Variable((self.sx, self.N+1))
        u = cvxpy.Variable((self.su, self.N))
        cost = 0.0
        constraints = []

        for t in range(self.N):
             # 4.29 마지막 실험에서 이거 빼고 함
            cost += cvxpy.quad_form(u[:, t], self.R) # minimize input
            if t != 0:
                cost += cvxpy.quad_form(Xref[:, t] - x[:, t], self.Q) # minimize ref error

            constraints += [x[:, t+1] == A_list[t]@x[:,t] + B_list[t]@u[:,t] + C_list[t]]

            if t < (self.N-1):
                cost += cvxpy.quad_form(u[:, t+1] - u[:, t], self.Rd) # minimize d(input)
                # 4.29 이거 의미없어보여서 걍 없앰 
                constraints += [cvxpy.abs(u[0,t+1] - u[0,t]) <= self.MAX_DSTEER*self.Ts]

        cost += cvxpy.quad_form(Xref[:, self.N] - x[:, self.N], self.Qf)

        constraints += [x[:, 0] == current_state]
        constraints += [cvxpy.abs(u[0, :]) <= self.MAX_STEER]
        constraints += [cvxpy.abs(u[1, :]) <= self.MAX_WW]
        prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        # prob.solve(solver=cvxpy.SCS, verbose=False)
        try:
            prob.solve(solver=cvxpy.OSQP, verbose=False)
        except cvxpy.error.SolverError:
            print("!!!")

        if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
            opt_x = self.get_nparray_from_matrix(x.value[0, :])
            opt_y = self.get_nparray_from_matrix(x.value[1, :])
            opt_phi = self.get_nparray_from_matrix(x.value[2, :])
            opt_vx = self.get_nparray_from_matrix(x.value[3, :])
            opt_vy = self.get_nparray_from_matrix(x.value[4, :])
            opt_yawrate = self.get_nparray_from_matrix(x.value[5, :])

            opt_delta = self.get_nparray_from_matrix(u.value[0, :])
            opt_ww = self.get_nparray_from_matrix(u.value[1, :])

            # self.opt_delta_pre, self.opt_ww_pre = opt_delta, opt_ww
        else:
            print("Error: Cannot solve mpc..")
            opt_delta, opt_ww = self.opt_delta_pre, self.opt_ww_pre
        opt_input = np.asarray([opt_delta, opt_ww])
        return opt_input

    def get_nparray_from_matrix(self, x):
        return np.array(x).flatten()