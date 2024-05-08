import numpy as np

class RefStateInuputCalculator:
    def __init__(self, ModelParams, MpcParams, PathParams):
        self.ModelParams = ModelParams
        self.MpcParams = MpcParams
        self.Pathparams = PathParams
        
        self.sx = self.ModelParams.sx

        self.N = self.MpcParams.N
        self.Ts = self.MpcParams.Ts

        self.list_x = self.Pathparams.list_x
        self.list_y = self.Pathparams.list_y
        self.list_phi = self.Pathparams.list_phi
        self.list_vx = self.Pathparams.list_vx
        self.list_vy = self.Pathparams.list_vy
        self.list_yawrate = self.Pathparams.list_yawrate
        self.length_index = self.Pathparams.length_index
        self.num_index = self.Pathparams.num_index

        self.stateindex_x = self.ModelParams.stateindex_x 
        self.stateindex_y = self.ModelParams.stateindex_y
        self.stateindex_phi = self.ModelParams.stateindex_phi
        self.stateindex_vx = self.ModelParams.stateindex_vx
        self.stateindex_vy = self.ModelParams.stateindex_vy
        self.stateindex_omega = self.ModelParams.stateindex_omega
        
        self.xref = np.zeros((self.sx, self.N + 1))

    def calc_ref_trajectory(self, current_state, current_index): # current_velocity -> m/s
        self.xref = np.zeros((self.sx, self.N + 1))
        current_vx, current_vy = current_state[3], current_state[4]
        current_velocity = np.sqrt(current_vx**2 + current_vy**2)
        # current_velocity = 2

        self.xref[0, 0] = current_state[self.stateindex_x]
        self.xref[1, 0] = current_state[self.stateindex_y]
        self.xref[2, 0] = current_state[self.stateindex_phi]
        self.xref[3, 0] = current_state[self.stateindex_vx]
        self.xref[4, 0] = current_state[self.stateindex_vy]
        self.xref[5, 0] = current_state[self.stateindex_omega]
        travel = 0.0

        for i in range(self.N + 1):
            travel += abs(current_velocity) * self.Ts
            dind = int(round(travel / self.length_index))

            if (current_index + dind) < self.num_index:
                self.xref[0, i] = self.list_x[current_index + dind]
                self.xref[1, i] = self.list_y[current_index + dind]
                self.xref[2, i] = self.list_phi[current_index + dind]
                self.xref[3, i] = self.list_vx[current_index + dind]
                self.xref[4, i] = self.list_vy[current_index + dind]
                self.xref[5, i] = self.list_yawrate[current_index + dind]

            else:
                self.xref[0, i] = self.list_x[current_index + dind - self.num_index - 1]
                self.xref[1, i] = self.list_y[current_index + dind - self.num_index - 1]
                self.xref[2, i] = self.list_phi[current_index + dind - self.num_index - 1]
                self.xref[3, i] = self.list_vx[current_index + dind - self.num_index - 1]
                self.xref[4, i] = self.list_vy[current_index + dind - self.num_index - 1]
                self.xref[5, i] = self.list_yawrate[current_index + dind - self.num_index - 1]

        return self.xref
