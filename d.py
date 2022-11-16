import random
import numpy as np
import math
from matplotlib import pyplot as plt
from global_var import *

"""
Simulering och tidssteg osv: 
"""

t = np.arange(0., T, dt)

from pid import PID

def main():
    spärr = False
    # PI-kontroller (deriveringen är känslig för noise)
    pid = PID(Kp, Ki, Kd, dt, (maxwmotor/k), -(maxwmotor/k)) # skapar regulatorn med konstanterna Kp = 0.005, Ki = 0.0002, Kd = 0.001 och max min
                                                                              # Elmotorns max rotations 
    tryck = 0
    v_ref = 0.0366

    for i in range(N):
        F_a = a_last[i]*m_last #accelerations kraften utanför för det är samma i alla lägen
        r_vinsch = 0.05*(l_vajer/(l_vajer + (s_last[i-1]*(1/3)))) # Funktion för vinschradiens förhållande till båtens position. (Förklaras i 'a' i rapport)
        
        if(s_last[i-1] < 6 and v_ref > 0): # Båten är på trailern och åker ner mot vattnet
            v_ref = 0.0366
            F_last[i] =(1 + (random.uniform(-1, 1))*0.01)*(F - F_f + F_a)

        elif(s_last[i-1] >= 6 and s_last[i-1] <= 8 and v_ref >0): # Båten åker ut i vattnet i 2 meter
            v_ref= 0.0366
            F_last[i] =(1 + (random.uniform(-1, 1))*0.01)*(F_a/math.cos(12))

        elif((s_last[i-1] > 8 and v_ref > 0) or (s_last[i-1] > 6 and v_ref < 0)): # Båten vänder riktning i vattnet och åker mot trailer
            
            v_ref = -0.0366
            F_last[i] =(1 + (random.uniform(-1, 1))*0.01)*(F_a/math.cos(12))

        else: #Båten är på trailern och dras upp
            F_last[i] =(1 + (random.uniform(-1, 1))*0.01)*(F + F_f + F_a)

            if tryck > 100 or s_last[i-1] < 0.1: # [Pa] om det är 10 cm kvar på så bromsar den in
                v_ref = 0
                I_motor[i] = 0

            else: # annars så drar den upp båten som vanligt
                v_ref = -0.0366
            

        if spärr == True: # om spärren är på -> stanna
            v_ref = 0
            I_motor[i] = 0

        # Kollar vridmomentet för att senare kunna ta ut strömmen
        # Men i vårt verkliga system kommer sensorn att göra det
        T_l[i] = F_last[i]*r_vinsch
        T_dev[i] = (T_l[i]/(k * f))

        w_motor[i] = (Ke * dt * U_motor[i] - dt * T_dev[i] * R + J * R * w_motor[i-1]) / (J * R + dt * Ke**2)
        
        if spärr == False:
            I_motor[i] = (J*(w_motor[i] - w_motor[i-1])/dt + T_dev[i])/Ke
        if abs(I_motor[i])> maxI: 
            # Om systemet vill få mer ström än vad elmotorn klarar av så stängs systemet av
            if abs(T_dev[i]/Ke) > maxI:
                spärr = True
            elif I_motor[i] > maxI:
                I_motor[i] = maxI
                w_motor[i] = (Ke * dt * U_motor[i] - dt * Ke * I_motor[i] * R + J * R * w_motor[i-1]) / (J * R + dt * Ke**2)
                U_motor[i] = (w_motor[i]*Ke + R*I_motor[i])
            else:
                I_motor[i] = -maxI
                w_motor[i] = (Ke * dt * U_motor[i] - dt * Ke * I_motor[i] * R + J * R * w_motor[i-1]) / (J * R + dt * Ke**2)
                U_motor[i] = (w_motor[i]*Ke + R*I_motor[i])

        

        
        # Här tar vi ut spänningen och strömmen från systemet med sensorer
        # och sen använder vi dessa värden för att göra en virituell sensor för att hålla koll på hastigheten.
        
        # Virtuella sensorn mäter hastighet på vajer
        w_last[i] = w_motor[i]/k
        v_last[i] = w_last[i]*r_vinsch

        
        # Här beräknas spänningen för nästa tidssteg med vår regulator
        if i < N-1:
            U_motor[i+1] = ((pid.update(v_last[i-1], v_ref)/r_vinsch)*k)*Ke + R*I_motor[i] 
            if U_motor[i+1] > maxU:
                U_motor[i+1] = maxU
            elif U_motor[i+1] < -maxU:
                U_motor[i+1] = -maxU

        if i > 0: #Eulers metod
            a_last[i] = (v_last[i] - v_last[i-1]) / dt
            s_last[i] = s_last[i-1] + dt*v_last[i]
    
    """
    Plotta grafer - hastighet, distans, acceleration, ström, spänning 
    """

    plt.figure(1)
    plt.plot(t, v_last)
    plt.xlabel('tid')
    plt.ylabel('hastighet [m/s]')

    plt.figure(2)
    plt.plot(t, s_last)
    plt.xlabel('tid')
    plt.ylabel('distans [m]')
    
    plt.figure(3)
    plt.plot(t, a_last)
    plt.xlabel('tid')
    plt.ylabel('acceleration [m/s^2]')

    
    plt.figure(4)
    plt.plot(t, I_motor)
    plt.xlabel('tid')
    plt.ylabel('ström [A]')

    plt.figure(5)
    plt.plot(t, U_motor)
    plt.xlabel('tid')
    plt.ylabel('spänning [V]')
    
    plt.show()


if __name__ == "__main__":
    main()
