class PID:
    def __init__(self, Kp = 0., Ki = 0., Kd = 0., dt = 0.001, max_u = None, min_u = None):
        # Skapar PID-regulatorn med de olika konstanterna, dt vilket är samplingsintervallet (/h), samt min och max spänning. 
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        self.dt = dt
        self.u = 0
        self.u0 = 0        
        self.max = max_u
        self.min = min_u
        self.prev__error = 0

    def update(self, input, r_ref):
        # Själva användandet av regulatorn. 

        error = r_ref - input
        P = self.Kp * error                                 # Kp*e  
        I = self.Ki * self.dt * error                       # Ki * \integral (e dt)
        D = self.Kd * (error - self.prev__error) / self.dt  # Kd * de/dt
        # self.u0 = self.u
        self.u = P + I + D                                  # Kp*e + Ki* \integral (e dt) + Kd * de/dt (precis som från föreläsningarna)

        if self.max is not None and self.u > self.max:      # Se till så att vi inte överskrider max/min spänning. 
            self.u = self.max
        elif self.min is not None and self.u < self.min:
            self.u = self.min
        self.prev__error = error
        return self.u                                       # den resulterade spänningen som vi skickar till motorn. 