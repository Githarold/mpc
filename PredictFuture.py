import numpy as np
from scipy.linalg import expm

class PredictFuture:
    def __init__(self, MpcParams, ModelParams):        
        self.ModelParams = ModelParams
        self.MpcParams = MpcParams

        self.N = self.MpcParams.N
        self.Ts = self.MpcParams.Ts

        self.sx = self.ModelParams.sx
        self.su = self.ModelParams.su
        
        self.m = self.ModelParams.m
        self.Iz = self.ModelParams.Iz
        self.lf = self.ModelParams.lf
        self.lr = self.ModelParams.lr
        self.r = self.ModelParams.r

        self.Cr = self.ModelParams.Cr
        self.Cf = self.ModelParams.Cf
        self.Cgr = self.ModelParams.Cgr

        self.stateindex_x = self.ModelParams.stateindex_x
        self.stateindex_y = self.ModelParams.stateindex_y
        self.stateindex_phi = self.ModelParams.stateindex_phi
        self.stateindex_vx = self.ModelParams.stateindex_vx
        self.stateindex_vy = self.ModelParams.stateindex_vy
        self.stateindex_omega = self.ModelParams.stateindex_omega

        self.inputindex_delta = self.ModelParams.inputindex_delta
        self.inputindex_ww = self.ModelParams.inputindex_ww

        self.MAX_STEER = self.ModelParams.MAX_STEER

        self.MAX_SPEED = self.ModelParams.MAX_SPEED
        self.MIN_SPEED = self.ModelParams.MIN_SPEED

        self.Ak_list = np.zeros((self.N, self.sx, self.sx))
        self.Bk_list = np.zeros((self.N, self.sx, self.su))
        self.gk_list = np.zeros((self.N, self.sx))

    # main of this class
    def predict_state(self, current_state, opt_input, Xref): # current_state, opt_input -> array
        Xbar_k = Xref * 0.0
        for i in range(self.sx):
            Xbar_k[i,0] = current_state[i]

        state = current_state
        for i, input in enumerate(opt_input): # i : 0 to 12, Xbar_k = 6,13
            next_state, Ak, Bk, gk = self.update(state, input)
            for j in range(self.sx):
                Xbar_k[j,i+1] = next_state[j]
            if i != self.N:
                self.Ak_list[i], self.Bk_list[i], self.gk_list[i] = Ak, Bk, gk # A,B,C -> channel : 0 ~ 12
            state = next_state  

        return Xbar_k, self.Ak_list, self.Bk_list, self.gk_list

    def update(self, Xk, Uk): # state : state array, input : delta
        # if Uk[0] >= self.MAX_STEER:
        #     Uk[0] = self.MAX_STEER  
        # elif Uk[0] <= -self.MAX_STEER:
        #     Uk[0] = -self.MAX_STEER

        # if Uk[1] >= self.MAX_SPEED:
        #     Uk[1] = self.MAX_SPEED
        # elif Uk[1] <= -self.MIN_SPEED:
        #     Uk[1] = -self.MIN_SPEED
        Ak, Bk, gk = self.DiscretizedLinearizedModel(Xk, Uk)
        Xk, Uk = np.asarray(Xk), np.asarray(Uk)

        next_state = Ak@Xk + Bk@Uk + gk
        next_state = next_state.reshape(6,)

        return next_state, Ak, Bk, gk

    def DiscretizedLinearizedModel(self, Xbar_k, Ubar_k):

        phi = Xbar_k[self.stateindex_phi]
        vx = Xbar_k[self.stateindex_vx]
        vy = Xbar_k[self.stateindex_vy]
        omega = Xbar_k[self.stateindex_omega]

        delta = Ubar_k[self.inputindex_delta]
        ww = Ubar_k[self.inputindex_ww]

        # if abs(ww) < 1/0.1375:
        #     ww = vx / self.r
        # 타이어 slip ratio 계산
        
        # if np.sqrt(vx**2 + vy**2) > -1:
        slip_ratio_f_y = vy/vx + self.lf/vx*omega - delta
        slip_ratio_r_y = vy/vx - self.lr/vx*omega
        if vx > self.r*ww:
            slip_ratio_r_x = 1 - (self.r*ww)/vx
        else:
            slip_ratio_r_x = 1 - vx/(self.r*ww)

        Ffy = -self.Cf*slip_ratio_f_y
        Fry = -self.Cr*slip_ratio_r_y
        Frx = self.Cgr*slip_ratio_r_x

        dFfy_dx = 0
        dFfy_dy = 0
        dFfy_dphi = 0
        dFfy_dvx = self.Cf*(vy + self.lf*omega)/vx**2
        dFfy_dvy = -self.Cf/vx
        dFfy_domega = -self.Cf*self.lf/vx
        dFfy_ddelta = self.Cf
        dFfy_dww = 0

        dFry_dx = 0
        dFry_dy = 0
        dFry_dphi = 0
        dFry_dvx = -self.Cr*(-vy+self.lr*omega)/vx**2
        dFry_dvy = -self.Cr/vx
        dFry_domega = self.Cr*self.lr/vx
        dFry_ddelta = 0
        dFry_dww = 0

        dFrx_dx = 0 
        dFrx_dy = 0 
        dFrx_dphi = 0 
        dFrx_dvx = -self.Cgr/(self.r*ww)
        dFrx_dvy = 0
        dFrx_domega = 0 
        dFrx_ddelta = 0
        dFrx_dww = self.Cgr*vx/(self.r*ww**2)

        # f1 = v_x*cos(phi) - v_y*sin(phi)
        df1_dx = 0
        df1_dy = 0
        df1_dphi = -vx*np.sin(phi) - vy*np.cos(phi)
        df1_dvx = np.cos(phi)
        df1_dvy = -np.sin(phi)
        df1_domega = 0
        df1_ddelta = 0
        df1_dww = 0

        # f2 = v_y*cos(phi) + v_x*sin(phi);
        df2_dx = 0
        df2_dy = 0
        df2_dphi =  vx*np.cos(phi) - vy*np.sin(phi)
        df2_dvx = np.sin(phi)
        df2_dvy = np.cos(phi)
        df2_domega = 0
        df2_ddelta = 0
        df2_dww = 0

        # f3 = omega
        df3_dx = 0
        df3_dy = 0
        df3_dphi = 0
        df3_dvx = 0
        df3_dvy = 0
        df3_domega = 1
        df3_ddelta = 0
        df3_dww = 0

        # f4 = 1/m*(F_rx - F_fy*sin(delta) + m*v_y*omega)
        df4_dx = 0
        df4_dy = 0
        df4_dphi = 0
        df4_dvx = 1/self.m*(dFrx_dvx - dFfy_dvx*np.sin(delta))
        df4_dvy = 1/self.m*(dFrx_dvy - dFfy_dvy*np.sin(delta) + self.m*omega)
        df4_domega = 1/self.m*(dFrx_domega - dFfy_domega*np.sin(delta) + self.m*vy)
        df4_ddelta = 1/self.m*(dFrx_ddelta - dFfy_ddelta*np.sin(delta) - Ffy*np.cos(delta))
        df4_dww = 1/self.m*(dFrx_dww - dFfy_dww*np.sin(delta))

        # f5 = 1/m*(F_ry + F_fy*cos(delta) - m*v_x*omega)
        df5_dx = 0
        df5_dy = 0
        df5_dphi = 0
        df5_dvx = 1/self.m*(dFry_dvx + dFfy_dvx*np.cos(delta) - self.m*omega)
        df5_dvy = 1/self.m*(dFry_dvy + dFfy_dvy*np.cos(delta))
        df5_domega = 1/self.m*(dFry_domega + dFfy_domega*np.cos(delta) - self.m*vx)
        df5_ddelta = 1/self.m*(dFry_ddelta + dFfy_ddelta*np.cos(delta) - Ffy*np.sin(delta))
        df5_dww = 1/self.m*(dFry_dww + dFfy_dww*np.cos(delta))

        # f6 = 1/Iz*(F_fy*l_f*cos(delta)- F_ry*l_r)
        df6_dx = 0
        df6_dy = 0
        df6_dphi = 0
        df6_dvx = 1/self.Iz*(dFfy_dvx*self.lf*np.cos(delta) - dFry_dvx*self.lr)
        df6_dvy = 1/self.Iz*(dFfy_dvy*self.lf*np.cos(delta) - dFry_dvy*self.lr)
        df6_domega = 1/self.Iz*(dFfy_domega*self.lf*np.cos(delta) - dFry_domega*self.lr)
        df6_ddelta = 1/self.Iz*(dFfy_ddelta*self.lf*np.cos(delta) - Ffy*self.lf*np.sin(delta) - dFry_ddelta*self.lr)
        df6_dww = 1/self.Iz*(dFfy_dww*self.lf*(np.cos(delta) - dFry_dww*self.lr))
        
        # else:
        #     # print("MIN V NO FORCE")
        #     slip_ratio_f_y = 0
        #     slip_ratio_r_y = 0
        #     slip_ratio_r_x = 0


        #     Ffy = 0
        #     Fry = 0
        #     Frx = 0

        #     dFfy_dx = 0
        #     dFfy_dy = 0
        #     dFfy_dphi = 0
        #     dFfy_dvx = 0
        #     dFfy_dvy = 0
        #     dFfy_domega = 0
        #     dFfy_ddelta = 0
        #     dFfy_dww = 0

        #     dFry_dx = 0
        #     dFry_dy = 0
        #     dFry_dphi = 0
        #     dFry_dvx = 0
        #     dFry_dvy = 0
        #     dFry_domega = 0
        #     dFry_ddelta = 0
        #     dFry_dww = 0

        #     dFrx_dx = 0 
        #     dFrx_dy = 0 
        #     dFrx_dphi = 0 
        #     dFrx_dvx = 0
        #     dFrx_dvy = 0
        #     dFrx_domega = 0 
        #     dFrx_ddelta = 0
        #     dFrx_dww = 0

        #     # f1 = v_x*cos(phi) - v_y*sin(phi)
        #     df1_dx = 0
        #     df1_dy = 0
        #     df1_dphi = -vx*np.sin(phi) - vy*np.cos(phi)
        #     df1_dvx = np.cos(phi)
        #     df1_dvy = -np.sin(phi)
        #     df1_domega = 0
        #     df1_ddelta = 0
        #     df1_dww = 0

        #     # f2 = v_y*cos(phi) + v_x*sin(phi);
        #     df2_dx = 0
        #     df2_dy = 0
        #     df2_dphi =  vx*np.cos(phi) - vy*np.sin(phi)
        #     df2_dvx = np.sin(phi)
        #     df2_dvy = np.cos(phi)
        #     df2_domega = 0
        #     df2_ddelta = 0
        #     df2_dww = 0

        #     # f3 = omega
        #     df3_dx = 0
        #     df3_dy = 0
        #     df3_dphi = 0
        #     df3_dvx = 0
        #     df3_dvy = 0
        #     df3_domega = 1
        #     df3_ddelta = 0
        #     df3_dww = 0

        #     # f4 = 1/m*(F_rx - F_fy*sin(delta) + m*v_y*omega)
        #     df4_dx = 0
        #     df4_dy = 0
        #     df4_dphi = 0
        #     df4_dvx = 1/self.m*(dFrx_dvx - dFfy_dvx*np.sin(delta))
        #     df4_dvy = 1/self.m*(dFrx_dvy - dFfy_dvy*np.sin(delta) + self.m*omega)
        #     df4_domega = 1/self.m*(dFrx_domega - dFfy_domega*np.sin(delta) + self.m*vy)
        #     df4_ddelta = 1/self.m*(dFrx_ddelta - dFfy_ddelta*np.sin(delta) - Ffy*np.cos(delta))
        #     df4_dww = 1/self.m*(dFrx_dww - dFfy_dww*np.sin(delta))

        #     # f5 = 1/m*(F_ry + F_fy*cos(delta) - m*v_x*omega)
        #     df5_dx = 0
        #     df5_dy = 0
        #     df5_dphi = 0
        #     df5_dvx = 1/self.m*(dFry_dvx + dFfy_dvx*np.cos(delta) - self.m*omega)
        #     df5_dvy = 1/self.m*(dFry_dvy + dFfy_dvy*np.cos(delta))
        #     df5_domega = 1/self.m*(dFry_domega + dFfy_domega*np.cos(delta) - self.m*vx)
        #     df5_ddelta = 1/self.m*(dFry_ddelta + dFfy_ddelta*np.cos(delta) - Ffy*np.sin(delta))
        #     df5_dww = 1/self.m*(dFry_dww + dFfy_dww*np.cos(delta))

        #     # f6 = 1/Iz*(F_fy*l_f*cos(delta)- F_ry*l_r)
        #     df6_dx = 0
        #     df6_dy = 0
        #     df6_dphi = 0
        #     df6_dvx = 1/self.Iz*(dFfy_dvx*self.lf*np.cos(delta) - dFry_dvx*self.lr)
        #     df6_dvy = 1/self.Iz*(dFfy_dvy*self.lf*np.cos(delta) - dFry_dvy*self.lr)
        #     df6_domega = 1/self.Iz*(dFfy_domega*self.lf*np.cos(delta) - dFry_domega*self.lr)
        #     df6_ddelta = 1/self.Iz*(dFfy_ddelta*self.lf*np.cos(delta) - Ffy*self.lf*np.sin(delta) - dFry_ddelta*self.lr)
        #     df6_dww = 1/self.Iz*(dFfy_dww*self.lf*(np.cos(delta) - dFry_dww*self.lr))
            

        # 상태 방정식
        f = np.array([vx*np.cos(phi) - vy*np.sin(phi),
                     vx*np.sin(phi) + vy*np.cos(phi),
                     omega,
                     1/self.m*(Frx - Ffy*np.sin(delta) + self.m*vy*omega),
                     1/self.m*(Fry + Ffy*np.cos(delta) - self.m*vx*omega),
                     1/self.Iz*(Ffy*self.lf*np.cos(delta) - Fry*self.lr)])

        # 타이어 힘 미분식
        

        Ac = np.array([[df1_dx, df1_dy, df1_dphi, df1_dvx, df1_dvy, df1_domega],
                       [df2_dx, df2_dy, df2_dphi, df2_dvx, df2_dvy, df2_domega],
                       [df3_dx, df3_dy, df3_dphi, df3_dvx, df3_dvy, df3_domega],
                       [df4_dx, df4_dy, df4_dphi, df4_dvx, df4_dvy, df4_domega],
                       [df5_dx, df5_dy, df5_dphi, df5_dvx, df5_dvy, df5_domega],
                       [df6_dx, df6_dy, df6_dphi, df6_dvx, df6_dvy, df6_domega]])        
        
        Bc = np.array([[df1_ddelta, df1_dww],
                       [df2_ddelta, df2_dww],
                       [df3_ddelta, df3_dww],
                       [df4_ddelta, df4_dww],
                       [df5_ddelta, df5_dww],
                       [df6_ddelta, df6_dww]])
        
        gc = f - np.dot(Ac, Xbar_k[:self.sx]) - np.dot(Bc, Ubar_k[:self.su])
        
        Bc_aug = np.hstack((Bc, gc.reshape(-1, 1)))
        
        # 이산화
        tmp = expm(np.block([[Ac, Bc_aug], [np.zeros((self.su+1, self.sx+self.su+1))]]) * self.Ts)
        
        Ad = np.zeros((self.sx, self.sx))
        Bd = np.zeros((self.sx, self.su))
        gd = np.zeros((self.sx, 1))
        
        Ad[:self.sx, :self.sx] = tmp[:self.sx, :self.sx]
        Bd[:self.sx, :self.su] = tmp[:self.sx, self.sx:self.sx+self.su]
        gd[:self.sx] = np.reshape(tmp[:self.sx, self.sx+self.su], (6,1))
        
        gd = gd.reshape(-1)

        return Ad, Bd, gd

