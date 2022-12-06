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

def main(verbose = True):
    spärr = False
    # PI-kontroller (deriveringen är känslig för noise)
    pid = PID(Kp, Ki, Kd, dt, (maxwmotor/k), -(maxwmotor/k)) # skapar regulatorn med konstanterna Kp = 0.005, Ki = 0.0002, Kd = 0.001 och max min
                                                                              # Elmotorns max rotations 
    tryck = 0
    v_ref = 0.0366
    no_boat = False
    i_cycle_switch = -1

    for i in range(N):
        F_a = a_last[i]*m_last #accelerations kraften utanför för det är samma i alla lägen
        r_vinsch = 0.05*(l_vajer/(l_vajer + (s_last[i-1]*(1/3)))) # Funktion för vinschradiens förhållande till båtens position. (Förklaras i 'a' i rapport)


        # Vi har lagt till ett brusfel med ett konstant fel på 0.1% plus ett slupmässigt mätfel mellan 0-0.1%.
        U_motor[i] = (1 + 0.001 + (random.uniform(0, 1))*0.001)*U_motor[i] # brusfelet blir random mellan 0.1-0.2%
        
        if(s_last[i-1] < 6 and v_ref > 0): # Båten är på trailern och åker ner mot vattnet
            v_ref = 0.0366
            F_last[i] =(F(no_boat) - F_f(no_boat) + F_a)

        elif(s_last[i-1] >= 6 and s_last[i-1] <= 8 and v_ref >0): # Båten åker ut i vattnet i 2 meter
            v_ref= 0.0366
            F_last[i] =(F_a/math.cos(12))

        elif((s_last[i-1] > 8 and v_ref > 0) or (s_last[i-1] > 6 and v_ref < 0)): # Båten vänder riktning i vattnet och åker mot trailer
            no_boat = not no_boat
            v_ref = -0.0366
            F_last[i] =(F_a/math.cos(12))

        else: #Båten är på trailern och dras upp
            F_last[i] =(F(no_boat) + F_f(no_boat) + F_a)
            
            # Om det är 10 cm kvar så bromsar den in pga vi inte har en riktig sensor just nu
            # För i riktiga systemet så behöver systemet bara stanna när båten trycker på töjningsgivaren
            if tryck > 100 or s_last[i-1] < 0.1: # [N] sensor när båt är uppe
                if no_boat:
                    v_ref = 0.0366
                    i_cycle_switch = i
                else:
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
        
        
        # Här tar vi ut spänningen och strömmen(Men får bli vridmoment eftersom vi inte har en riktig sensor) från systemet
        # med sensorer och sen använder vi dessa värden för att göra en virituell sensor för att få ut hastigheten.
        w_motor[i] = (Ke * dt * U_motor[i] - dt * T_dev[i] * R + J * R * w_motor[i-1]) / (J * R + dt * Ke**2)
        
        if spärr == False:
            I_motor[i] = (J*(w_motor[i] - w_motor[i-1])/dt + T_dev[i])/Ke
            # Vi har lagt till ett brusfel med ett konstant fel på 0.2% plus ett slupmässigt mätfel mellan -0-1% - 0.1%.
            I_motor_measured[i] = (1 + 0.002 + (random.uniform(-1, 1))*0.001)*I_motor[i] # brusfelet blir random mellan 0.1-0.3%

        if abs(I_motor_measured[i])> maxI: 
            # Om systemet vill få mer ström än vad elmotorn klarar av så stängs systemet av
            spärr = True
        
        
        # Virtuella sensorn mäter hastighet på vajer
        w_last[i] = w_motor[i]/k
        v_last[i] = w_last[i]*r_vinsch

        #Energiförbrukning
        P_last[i] = (F_last[i]*v_last[i])
        P_motor[i] =(T_dev[i]*w_motor[i])
        P_batt[i] =(I_motor[i]*U_motor[i])
        I_batt[i] =(P_batt[i]/U_batt)
        
        # Här beräknas spänningen för nästa tidssteg med vår regulator
        if i < N-1:
            # skickar in alla parametrar som behövs för att få ut den nya spänningen till regulatorn.
            U_motor[i+1] = pid.update(v_last[i], v_ref, r_vinsch ,k ,Ke ,R ,I_motor[i])

            #Om förändringen i spänning blir för stor så önskar systemet mer ström än vad motorn klarar av
            #men då ändras spänningen så att strömmen blir till sin max strm istället.
            if U_motor[i+1] > (((Ke * dt * U_motor[i+1] - dt * Ke * maxI * R + J * R * w_motor[i]) / (J * R + dt * Ke**2))*Ke + R*maxI):
                U_motor[i+1] = (((Ke * dt * U_motor[i+1] - dt * Ke * maxI * R + J * R * w_motor[i]) / (J * R + dt * Ke**2))*Ke + R*maxI)
            
            elif U_motor[i+1] < (((Ke * dt * U_motor[i+1] - dt * Ke * -maxI * R + J * R * w_motor[i]) / (J * R + dt * Ke**2))*Ke + R*-maxI):
                U_motor[i+1] = (((Ke * dt * U_motor[i+1] - dt * Ke * -maxI * R + J * R * w_motor[i]) / (J * R + dt * Ke**2))*Ke + R*-maxI)

        if i > 0: #Eulers metod
            a_last[i] = (v_last[i] - v_last[i-1]) / dt
            s_last[i] = s_last[i-1] + dt*v_last[i]
        v_last_ref[i] = v_ref


    E_total_1 = sum(abs(P_batt[:i_cycle_switch])) * dt
    E_total_2 = sum(abs(P_batt[i_cycle_switch:])) * dt

    if not verbose:
        return E_total_1, E_total_2
    print(f"Total energi användningscykel 1: {round(E_total_1/1000, 3)} kJ")
    print(f"Total energi användningscykel 2: {round(E_total_2/1000, 3)} kJ")

    """
    Plotta grafer - hastighet, distans, acceleration, ström, spänning 
    """
    plt.figure(1) # plot av hastigheten beroende på tiden
    plt.plot(t, v_last)
    plt.xlabel('tid')
    plt.ylabel('hastighet [m/s]')

    plt.figure(2) # plot av sträckan beroende på tiden
    plt.plot(t, s_last)
    plt.xlabel('tid')
    plt.ylabel('distans [m]')
    
    plt.figure(3) # plot av accelerationen beroender på tiden
    plt.plot(t, a_last)
    plt.xlabel('tid')
    plt.ylabel('acceleration [m/s^2]')
    
    plt.figure(4) # plot av strömmen i motorn beroende på tiden
    plt.plot(t, I_motor)
    plt.xlabel('tid')
    plt.ylabel('ström [A]')

    plt.figure(5) # plot av strömmen i motorn beroende på tiden
    plt.plot(t, U_motor)
    plt.xlabel('tid')
    plt.ylabel('spänning [V]')

    plt.figure(6)
    plt.plot(t, P_batt)
    plt.xlabel('tid')
    plt.ylabel('Batteri effekt [W]')

    plt.figure(7)
    plt.plot(t, P_motor)
    plt.xlabel('tid')
    plt.ylabel('Motorns effekt [W]')

    plt.figure(8)
    plt.plot(t, P_last)
    plt.xlabel('tid')
    plt.ylabel('Last effekt [W]')

    plt.figure(9)
    plt.plot(t, F_last)
    plt.xlabel('tid')
    plt.ylabel('F last [N]')
    
    plt.show()
    return E_total_1, E_total_2


if __name__ == "__main__":
    calculate_energy_consumption = False

    if not calculate_energy_consumption:
        main(True)

    else:
        E1s = []
        E2s = []

        n = 1000
        for i in range(n):
            E1, E2 = main(False)
            E1s.append(E1)
            E2s.append(E2)
            print(f"iteration: {i}\tenergi 1: {round(E1/1000, 3)} kJ\tenergi 2: {round(E2/1000, 3)} kJ")

        print()
        for i, Es in enumerate([E1s, E2s]):
            print("Användningscykel:", i+1)
            averageE = sum(Es)/len(Es)
            maxE = max(Es)
            minE = min(Es)

            print(f"Medel-energiförbrukning: {round(averageE/1000, 3)} kJ")
            print(f"Max-energiförbrukning: {round(maxE/1000, 3)} kJ")
            print(f"Min-energiförbrukning: {round(minE/1000, 3)} kJ")
            deltaE = maxE-averageE if maxE-averageE > averageE-minE else averageE-minE
            print(f"Mätosäkerhet: {round(deltaE/1000, 3)} kJ = {round(deltaE/averageE*100, 2)}%")
            print()