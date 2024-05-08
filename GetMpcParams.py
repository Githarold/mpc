import numpy as np

class MpcParams:
    def __init__(self):
        self.Model = None # mpc model name
        self.N = None # prediction horizon
        self.Ts = None # sampling time
        # self.max_iter = None # max iteration

        self.R = None
        self.Rd = None
        self.Q = None
        self.Qf = None

class ModelConverter:
    @staticmethod
    def getMpcParams(Model):

        mpc_params = MpcParams()

        if Model == 'LMPC_tracking':

            mpc_params.Model = 'LMPC_tracking'
            mpc_params.N = 13
            mpc_params.Ts = 0.1

            mpc_params.R = np.diag([0.01, 0.01])  # input cost matrix
            mpc_params.Rd = np.diag([1, 0.1])  # input difference cost matrix
            mpc_params.Q = np.diag([1.0, 1.0, 10.0, 10.0, 0.5, 1])  # state cost matrix
            mpc_params.Qf = mpc_params.Q  # state final matrix

        else:
            raise ValueError('Model invalid')
        
        return mpc_params