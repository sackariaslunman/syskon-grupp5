import math
import numpy as np

"""
Motorns värden:
"""
maxT_dev = 0.5      # [Nm]  Högsta utvecklade vridmoment. 
maxwmotor = 502.4   # [rad/s] Högsta vinkelhastigheten hos motorn. 
maxP = 251.2        # [W] Max utvecklad effekt
maxU = 10.67        # [V] Max spänning ur motorn
maxI = 28.8         # [I] Max ström genom motorn. 
R = 0.0459          # [Ohm] Inre resistans hos motorn. 
Ke = 0.0184         # Spänningskonstanten i motorn. Antas vara samma som Ki. 
J = 0.1

"""
Växellådans värden:
"""
k = 3.5**5          # 1 : 525.2  i uväxling.
f = 0.95**5         # Förluster i växellådan. 5 % per steg. 

"""
Trailer och last (gravitation med): 
"""
m_last = 1500 #* 0.5
g = 9.82            # [m/s^2] gravitationskonstanten.
w_last_ref = 0.733  # [w] rad/s
s_total = 6.6       # [m] Meter vilket är den totala sträckan båten kommer färdas i. 
simulation_duration = 180 # [s] sekunder
l_vajer = 8.        # [m] Meter. 


"""
Beräknade värden: 
"""
F = m_last * g * math.sin(12*2*math.pi/360) #vertikaltlyft
cr = 0.05 # Friktionskonstanten
F_f = cr * m_last * g * math.cos(12*2*math.pi/360)       # Friktionskraften. 
v_last = s_total*simulation_duration


dt = 0.01               # [s], sekunder per tidssteg för simulering
T = 500.0               # [s], sekunder för hela simuleringen
N = round (T/dt)        # antal steg att simulera avrundat till heltal


v_last = np.zeros(N)    # [m/s]     Hastighet på vinsch - begynelsevärde
a_last = np.zeros(N)    # [m/s^2]   Acceleration på vinsch - begynelsevärd
s_last = np.zeros(N)    # [m]       Sträckan på last - begynelsevärde
spärr = bool # det som bestämmer om systemet skall stanna eller ej. Bestäms genom trycksensor. 
F_last = np.zeros(N)    # [N]       Kraften lasten verkar med på systemet / vinschen. 
w_motor = np.zeros(N)   # [rad/s]   Rotationshastigheten motorn.
T_l = np.zeros(N)       # [Nm]      Det utvecklade momentet från systemet och lsaten på vinschen. 
T_dev = np.zeros(N)     # [Nm]      Motorns utvecklade moment. 
w_motor = np.zeros(N)
w_last = np.zeros(N)    # [rad/s]   Rotationshastigheten vinschen
P_motor = np.zeros(N)   # [W]       Effekt ur motorn. 
I_motor = np.zeros(N)   # [A]       Ström genom motorn
U_motor = np.zeros(N)   # [V]       Spänning över motorn 
P_batt = np.zeros(N)    # [W]       Effekten som batteriet förser
W_batt = np.zeros(N)    # [W]       Effekt av något. 
I_batt = np.zeros(N)    # [A]       Strömmen som batteriet förser
U_batt = 12             # [V]       Spänningen som batteriet alltid förser. 
v_ref = 0.0366          # [m/s]     Referenssignalen till PI - regulatorn. 
Kp = 0.1                # Propotionella konstanten av en regulator. 'P' delen. 
Ki = 1                  # Integrala konstanten av en regulator. 'I' delen. 
Kd = 0                  # Deriverade konstanten av en regulator 'D' delen. Satt till 0 då vi 
                        # bara har PI regulator. (Vi behöver inget PID)