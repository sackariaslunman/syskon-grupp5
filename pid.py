class PID:
    def __init__(self, Kp = 0, Ki = 0, Kd = 0, h = 0.001, goal = 0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.u = 0
        self.u0 = 0
        self.h = h
        self.goal = goal

    def update(self, input):
        error = self.goal - input
        P = self.Kp * error
        I = self.u + self.Ki * self.h * error
        D = self.Kd * (self.u - self.u0) / self.h
        self.u0 = self.u
        self.u = P + I + D
        return self.u