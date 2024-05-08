import math

class ModelParams:
    def __init__(self):
        self.Model = None

        self.sx = None # number of states
        self.su = None # number of inputs

        self.stateindex_x = None # x position
        self.stateindex_y = None # y position
        self.stateindex_phi = None # orientation
        self.stateindex_vx = None # longitudinal velocity
        self.stateindex_vy = None # later velocity
        self.stateindex_omega = None # yaw rate

        self.inputindex_delta = None # steering angle
        self.inputindex_ww = None # anglular velocity (motor -> wheel)

        self.m = None # mass (kg)
        self.Iz = None # inertia (kg^3)
        self.lf = None # cg2frontwheel (m)
        self.lr = None # cg2backwheel (m)
        self.r = None # wheel radius (m)

        self.Cr = None # slip & rear tire lateral force slope
        self.Cf = None # slip & front tire lateral force slope
        self.Cgr = None # slip & rear tire longitudinal force slope

        self.MAX_STEER = None  # maximum steering angle [rad]
        self.MAX_DSTEER = None  # maximum steering speed [rad/s]
        self.MAX_SPEED = None  # maximum speed [m/s]
        self.MIN_SPEED = None  # minimum speed [m/s]
        # self.MAX_ACCEL = 1.0  # maximum accel [m/ss]
        self.MAX_WW = None# need to check
        self.MIN_WW = None


class ModelConverter:
    @staticmethod
    def getModelParams(Model):

        model_params = ModelParams()

        if Model == 'hunter':

            model_params.Model = 'hunter'

            model_params.sx = 6
            model_params.su = 2

            model_params.stateindex_x = 0
            model_params.stateindex_y = 1
            model_params.stateindex_phi = 2
            model_params.stateindex_vx = 3
            model_params.stateindex_vy = 4 
            model_params.stateindex_omega = 5

            model_params.inputindex_delta = 0
            model_params.inputindex_ww = 1

            model_params.m = 47.4
            model_params.Iz = 0.450
            model_params.lf = 0.341
            model_params.lr = 0.208
            model_params.r = 0.1375

            model_params.Cr = 38.78
            model_params.Cf = 36.26
            model_params.Cgr = 2829.37

            model_params.MAX_STEER = math.radians(22.0)  # maximum steering angle [rad]
            model_params.MAX_DSTEER = math.radians(2.5)  # maximum steering speed [rad/s]
            model_params.MAX_SPEED = 4.8 # maximum speed [m/s]
            model_params.MIN_SPEED = 3.8  # minimum speed [m/s]
            model_params.MAX_WW = model_params.MAX_SPEED/model_params.r
            model_params.MIN_WW = model_params.MIN_SPEED/model_params.r

        else:
            raise ValueError('Model invalid')
        
        return model_params