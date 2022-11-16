import numpy as np
import math
from matplotlib import pyplot as plt
from global_var import *

"""
Simulering och tidssteg osv: 
# """

t = np.arange(0., T, dt)

from pid import PID
def main():
    for i in range(N):
        F_a = a_last[i]*m_last #accelerations kraften utanför för det är samma i alla lägen
        r_vinsch = 0.05*(l_vajer/(l_vajer + (s_last[i-1]*(1/3)))) # Funktion för vinschradiens förhållande till båtens position. (Förklaras i 'a' i rapport)
        F_last[i] =(F + F_f + F_a)

        # Kollar vridmomentet för att senare kunna ta ut strömmen
        # Men i vårt verkliga system kommer sensorn att göra det
        T_l[i] = F_last[i]*r_vinsch 
        T_dev[i] = (T_l[i]/(k * f))

        w_motor[i] = (Ke * dt * U_motor[i] - dt * T_dev[i] * R + J * R * w_motor[i-1]) / (J * R + dt * Ke**2)
        I_motor[i] = (J*(w_motor[i] - w_motor[i-1]) + T_dev[i])/Ke
        # Här tar vi ut spänningen och strömmen från systemet med sensorer
        # och sen använder vi dessa värden för att göra en virituell sensor för att hålla koll på hastigheten.
        
        # Virtuella sensorn mäter hastighet på vajer
        w_last[i] = w_motor[i]/k
        v_last[i] = w_last[i]*r_vinsch

        
        # Här beräknas spänningen för nästa tidssteg med vår regulator
        if i < N-1:
            U_motor[i+1] = ((maxU * 0.5 /r_vinsch)*k)*Ke + R*I_motor[i] 
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
    
    
    print("Max hastighet:", np.amax(v_last))
    print("Max acceleration:", np.amax(a_last))
    print("Max ström:", np.amax(I_motor))
    plt.show()


if __name__ == "__main__":
    main()
