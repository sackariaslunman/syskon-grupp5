import numpy as np
import math
from matplotlib import pyplot as plt
import global_var as gl

"""
Simulering och tidssteg osv: 
# """

t = np.arange(0., gl.T, gl.dt)

from pid import PID

def main():
    spärr = False
    pid = PID(gl.Kp, gl.Ki, gl.dt, (gl.maxwmotor/gl.k), -(gl.maxwmotor/gl.k)) # skapar regulatorn med konstanterna Kp = 0.005, Ki = 0.0002, Kd = 0.001 och max min
                                                                              # Elmotorns max rotations 
    tryck = 0
    gl.v_ref = 0.0366
    for i in range(gl.N):
        F_a = gl.a_last[i]*gl.m_last #accelerations kraften utanför för det är samma i alla lägen
        r_vinsch = 0.05*(gl.l_vajer/(gl.l_vajer + (gl.s_last[i-1]*(1/3)))) # Funktion för vinschradiens förhållande till båtens position. (Förklaras i 'a' i rapport)
        
        if(gl.s_last[i-1] < 6 and gl.v_ref > 0): # Båten är på trailern och åker ner mot vattnet
            gl.v_ref = 0.0366
            gl.F_last[i] =(gl.F - gl.F_f + F_a)

        elif(gl.s_last[i-1] >= 6 and gl.s_last[i-1] <= 8 and gl.v_ref >0): # Båten åker ut i vattnet i 2 meter
            gl.v_ref= 0.0366
            gl.F_last[i] =(F_a/math.cos(12))

        elif((gl.s_last[i-1] > 8 and gl.v_ref > 0) or (gl.s_last[i-1] > 6 and gl.v_ref < 0)): # Båten vänder riktning i vattnet och åker mot trailer
            
            gl.v_ref = -0.0366
            gl.F_last[i] =(F_a/math.cos(12))

        else: #Båten är på trailern och dras upp
            gl.F_last[i] =(gl.F + gl.F_f + F_a)

            if tryck > 100 or gl.s_last[i-1] < 0.1: # [Pa] om det är 10 cm kvar på så bromsar den in
                gl.v_ref = 0
                gl.I_motor[i] = 0

            else: # annars så drar den upp båten som vanligt
                gl.v_ref = -0.0366
            

        if spärr == True: # om spärren är på
            gl.v_ref = 0
            gl.I_motor[i] = 0


        gl.T_l[i] = gl.F_last[i]*r_vinsch 
        gl.T_dev[i] = (gl.T_l[i]/(gl.k * gl.f))

        if spärr == False:
            gl.I_motor[i] = gl.T_dev[i]/gl.Ke
        if gl.T_dev[i]/gl.Ke > gl.maxI: 
            # Om systemet vill få mer ström än vad elmotorn klarar av så stängs systemet av
            spärr = True
        

        gl.w_motor[i] = (gl.U_motor[i] - gl.R*gl.I_motor[i])/gl.Ke
        # Här tar vi ut spänningen och strömmen från systemet med sensorer
        # och sen använder vi dessa värden för att göra en virituell sensor för att hålla koll på hastigheten.
        
        gl.w_last[i] = gl.w_motor[i]/gl.k
        gl.v_last[i] = gl.w_last[i]*r_vinsch

        
        
        if i < gl.N-1:
            gl.U_motor[i+1] = ((pid.update(gl.v_last[i-1], gl.v_ref)/r_vinsch)*gl.k)*gl.Ke + gl.R*gl.I_motor[i] # Får ut fel värde just nu, vi vill få spänning men får v_ref istället
            if gl.U_motor[i+1] > gl.maxU:
                gl.U_motor[i+1] = gl.maxU
            elif gl.U_motor[i+1] < -gl.maxU:
                gl.U_motor[i+1] = -gl.maxU

        if i > 0: #Eulers metod
            gl.a_last[i] = (gl.v_last[i] - gl.v_last[i-1]) / gl.dt
            gl.s_last[i] = gl.s_last[i-1] + gl.dt*gl.v_last[i]
            gl.W_batt[i] = gl.W_batt[i-1] + gl.dt*gl.P_batt[i]


    plt.figure(1)
    plt.plot(t, gl.v_last)
    plt.xlabel('tid')
    plt.ylabel('hastighet [m/s]')

    plt.figure(2)
    plt.plot(t, gl.s_last)
    plt.xlabel('tid')
    plt.ylabel('distans [m]')
    
    plt.figure(3)
    plt.plot(t, gl.a_last)
    plt.xlabel('tid')
    plt.ylabel('acceleration [m/s^2]')

    
    plt.figure(4)
    plt.plot(t, gl.I_motor)
    plt.xlabel('tid')
    plt.ylabel('ström [A]')

    plt.figure(5)
    plt.plot(t, gl.U_motor)
    plt.xlabel('tid')
    plt.ylabel('spänning [V]')
    
    plt.show()


if __name__ == "__main__":
    main()
