class State:
    def __init__(self, x = 0.0, y = 0.0, phi = 0.0, vx = 0.0, vy = 0.0, yawrate = 0.0):
        self.x = x
        self.y = y
        self.phi = phi
        self.vx = vx
        self.vy = vy
        self.yawrate = yawrate

class Input:
    def __init__(self, delta):
        self.delta = delta