import numpy as np
from scipy.linalg import expm

class DiscretizedLinearizedModel:
    def __init__(self, ModelParams, Ts):

        self.ModelParams = ModelParams
        self.Ts = Ts
        
        # 상태 및 입력 변수 인덱스
        self.sx = self.ModelParams.sx - 1
        self.su = self.ModelParams.su - 1
        
        # 모델 파라미터
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

    def DiscretizedLinearizedModel(self, Xbar_k, Ubar_k, ww):
        
        # 상태 및 입력 변수 추출
        phi = Xbar_k[self.stateindex_phi]
        vx = Xbar_k[self.stateindex_vx]
        vy = Xbar_k[self.stateindex_vy]
        omega = Xbar_k[self.stateindex_omega]

        delta = Ubar_k[self.inputindex_delta]
        
        ###################몰루
        # # 속도 제한 조건
        # if vx < 0.5:
        #     vx = vx
        #     vy = 0
        #     omega = 0
        #     delta = 0
        #     if vx < 0.3:
        #         vx = 0.3
        
        # 타이어 슬립 각도 계산
        # alpha_f = -np.arctan2(self.lf*omega + vy, vx) + delta
        # alpha_r = np.arctan2(self.lr*omega - vy, vx)
        ###################

        # 타이어 slip ratio 계산
        slip_ratio_f_y = vy/vx + self.lf/vx*omega - delta
        slip_ratio_r_y = vy/vx - self.lr/vx*omega
        slip_ratio_r_x = 1 - vx/(self.r*ww) ## ww값은 직접 인자로 전달받아야댐 
        
        # 타이어 힘 계산
        # Ffy = self.Df*np.sin(self.Cf*np.arctan(self.Bf*alpha_f))
        # Fry = self.Dr*np.sin(self.Cr*np.arctan(self.Br*alpha_r))
        # Frx = self.Cm1*D - self.Cm2*D*vx - self.Cr0 - self.Cr2*vx**2
        Ffy = -self.Cf*slip_ratio_f_y
        Fry = -self.Cr*slip_ratio_r_y
        Frx = self.Cgr*slip_ratio_r_x
        
        # 상태 방정식
        f = np.array([vx*np.cos(phi) - vy*np.sin(phi),
                     vy*np.cos(phi) + vx*np.sin(phi),
                     omega,
                     1/self.m*(Frx - Ffy*np.sin(delta) + self.m*vy*omega),
                     1/self.m*(Fry + Ffy*np.cos(delta) - self.m*vx*omega),
                     1/self.Iz*(Ffy*self.lf*np.cos(delta) - Fry*self.lr)])

        # 타이어 힘 미분식
        dFfy_dvx = -self.Cf*(vy + self.lf*omega)/vx**2
        dFfy_dvy = -self.Cf/vx
        dFfy_domega = -self.Cf*self.lf/vx
        dFfy_ddelta = self.Cf
        dFfy_dww = 0

        dFry_dvx = -self.Cr*(-vy+self.lr*omega)/vx**2
        dFry_dvy = -self.Cr/vx
        dFry_domega = self.Cr*self.lr/vx
        dFry_ddelta = 0
        dFry_dww = 0

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
        df4_dvy = 1/self.m*(-dFfy_dvy*np.sin(delta) + self.m*omega)
        df4_domega = 1/self.m*(dFrx_domega - dFfy_domega*np.sin(delta) + self.m*vy)
        df4_ddelta = 1/self.m*(-self.Cf*np.sin(delta) - Ffy*np.cos(delta))
        df4_dww = 1/self.m*(dFrx_dww - dFfy_dww*np.sin(delta))

        # f5 = 1/m*(F_ry + F_fy*cos(delta) - m*v_x*omega)
        df5_dx = 0
        df5_dy = 0
        df5_dphi = 0
        df5_dvx = 1/self.m*(dFry_dvx + dFfy_dvx*np.cos(delta) - self.m*omega)
        df5_dvy = 1/self.m*(dFry_dvy + dFfy_dvy*np.cos(delta))
        df5_domega = 1/self.m*(dFry_domega + dFfy_domega*np.cos(delta) - self.m*vx)
        df5_ddelta = 1/self.m*(self.Cf*np.cos(delta) - Ffy*np.sin(delta))
        df5_dww = 1/self.m*(dFry_dww + dFfy_dww*np.cos(delta))

        # f6 = 1/Iz*(F_fy*l_f*cos(delta)- F_ry*l_r)
        df6_dx = 0
        df6_dy = 0
        df6_dphi = 0
        df6_dvx = 1/self.Iz*(dFfy_dvx*self.lf*np.cos(delta) - dFry_dvx*self.lr)
        df6_dvy = 1/self.Iz(dFfy_dvy*self.lf*np.cos(delta) - dFry_dvx*self.lr)
        df6_domega = 1/self.Iz*(dFfy_domega*self.lf*np.cos(delta) - dFry_domega*self.lr)
        df6_ddelta = 1/self.Iz*(self.Cf*self.lf*np.cos(delta) - Ffy*self.lf*np.sin(delta))
        df6_dww = 1/self.Iz*(dFfy_dww*self.lf(np.cos(delta) - dFry_dww*self.lr))

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
        
        gc = f - np.dot(Ac, self.Xbar_k[:self.sx]) - np.dot(Bc, self.Ubar_k[:self.su])
        
        Bc_aug = np.hstack((Bc, gc.reshape(-1, 1)))
        
        # 이산화
        tmp = expm(np.block([[Ac, Bc_aug], [np.zeros((self.su+1, self.sx+self.su+1))]]) * self.Ts)
        
        Ad = np.zeros((self.sx+1, self.sx+1))
        Bd = np.zeros((self.sx+1, self.su+1))
        gd = np.zeros((self.sx+1, 1))
        
        Ad[:self.sx, :self.sx] = tmp[:self.sx, :self.sx]
        Bd[:self.sx, :self.su] = tmp[:self.sx, self.sx:self.sx+self.su]
        gd[:self.sx] = tmp[:self.sx, self.sx+self.su]
        
 
        # 수치 오류 방지
        Ad[-1, -1] = 1
        Bd[-1, -1] = self.Ts
        
        return Ad, Bd, gd
