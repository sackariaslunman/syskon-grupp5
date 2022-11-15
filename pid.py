class PID:
    def __init__(self, Kp = 0., Ki = 0., dt = 0.001, max_u = None, min_u = None):
        # Skapar PID-regulatorn med de olika konstanterna, dt vilket är samplingsintervallet (/h), samt min och max spänning. 
        self.Kp = Kp
        self.Ki = Ki
        
        self.dt = dt
        self.u = 0
        self.u0 = 0        
        self.max = max_u
        self.min = min_u
        self.prev__error = 0
        self.prev_ref = 0
        self.new_ref = 0
        self.I_prev = 0
    def update(self, input, r_ref):
        # Själva användandet av regulatorn. 
        
        if self.prev_ref != r_ref:
            self.new_ref = 0
        
        if self.new_ref < 0.99:
            self.new_ref += 0.001
        else:
            self.new_ref = 1

        
        error = self.new_ref*r_ref - input
        
        P = self.Kp * error                                 # Kp*e  
        I = self.Ki *(self.I_prev + self.dt * error)
        self.I_prev = I                      # Ki * \integral (e dt)
        # D = self.Kd * (error - self.prev__error) / self.dt  Kd * de/dt
        # self.u0 = self.u
        self.u = P + I # Kp*e + Ki* \integral (e dt) + Kd * de/dt (precis som från föreläsningarna)
        if self.u > self.max:      # Se till så att vi inte överskrider max/min spänning. 
            self.u = 0.001
        elif self.u < self.min:
            self.u = 0.001
        self.prev__error = error
        self.prev_ref = r_ref
        return self.u                                       # den resulterade spänningen som vi skickar till motorn. 