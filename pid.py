class PID:
    def __init__(self, Kp = 0., Ki = 0., Kd = 0., h = 0.001, goal = 0., max = None, min = None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.u = 0
        self.u0 = 0
        self.h = h
        self.goal = goal
        self.max = max
        self.min = min

    def update(self, input):
        error = self.goal - input
        P = self.Kp * error
        I = self.u + self.Ki * self.h * error
        D = self.Kd * (self.u - self.u0) / self.h
        self.u0 = self.u
        self.u = P + I + D

        if self.max is not None and self.u > self.max:
            self.u = self.max
        elif self.min is not None and self.u < self.min:
            self.u = self.min

        return self.u