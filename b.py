import numpy as np
import math
from matplotlib import pyplot as plt

#motorns variabler
maxTDevMotor = 0.5 #[Nm]
maxWmotor = 502.4 #[w] rad/s
maxEffekt = 251.2 #[w] watt
maxU = 10.67 #[U] spänning
maxI = 28.8 # [i] ström
resistans = 0.0459 # [R] resistans
spänningskonstant = 0.0184
utväxling = 3.5**5 # 1 : 525.2
förluster = 0.95**5

#Ytterliggare variabler
MLast = 1500
g = 9.82 #gravitation
#maxTLast = 189.88 # [Nm]
w_last_ref = 0.733 # [w] rad/s
distance = 6.6 # [m] meter
maxTime = 180 #[s] sekunder
#vinschRadie = 0.05 # [m] meter
vajer_dist = 8. # [m] meter

#Uträknade siffror
#TDev = maxTLast/(utväxling * förluster) #0.4672 Nm
#wMotor = wVinsch * utväxling # [w] 385 rad/s
tyngdkraften = MLast * g * math.sin(12*2*math.pi/360) #vertikaltlyft
cr = 0.05 # Friktionskonstanten
friktionskraften = cr * MLast * g * math.cos(12*2*math.pi/360) #normalkraften
vLast = distance*maxTime


#modell av lasten
#FLast = tyngdkraften + friktionskraften + accelerationskraften
#PLast = vLast*FLast

#Inparametrar

dt = 0.01 #[s], sekunder per tidssteg för simulering
T = 500.0 # [s], sekunder för hela simuleringen
N = round (T/dt) #antal steg att simulera avrundat till heltal

v_last = np.zeros(N) #hastighet på vinsch - begynelsevärde
a_last = np.zeros(N) #acceleration på vinsch - begynelsevärd
s_last = np.zeros(N) #sträckan på last - begynelsevärde


F_last = np.zeros(N)
w_last = np.zeros(N)
w_motor = np.zeros(N)
T_l = np.zeros(N)
T_dev = np.zeros(N)
v_last = np.zeros(N)
w_last = np.zeros(N)
w_motor = np.zeros(N)
P_motor = np.zeros(N)
I_motor = np.zeros(N)
U_motor = np.zeros(N)
P_batt = np.zeros(N)
I_batt = np.zeros(N)
U_batt = 12 # [v] Volt - konstant
W_batt = np.zeros(N)
r_ref = 0.073   #referenssignal till systemet. Denna ska ändras beroende på vilket steg vi är i simuleringen. Baklänges / framlänges / stopp etc. 

t = np.arange(0., 500., dt)

F_last =(tyngdkraften - friktionskraften)

# Oklart vad J ska vara
J = 0.1

from pid import PID

def main():
    F_last =(tyngdkraften - friktionskraften) * 1       # faktorn representerar 100 %. 
    pid = PID(0.005, 0.0002, 0.001, dt, U_batt, -U_batt)        # skapar regulatorn med konstanterna Kp = 0.005, Ki = 0.0002, Kd = 0.001 och max min
                                                                # spänning är det batteriet kan förse som högst. 
    for i in range(N):
        vinschRadie = 0.05*(vajer_dist/(vajer_dist + (s_last[i-1]*(1/3)))) # Funktion för vinschradiens förhållande till båtens position. (Förklaras i 'a' i rapport)

        T_l[i] = F_last * vinschRadie                           # Vridmoment från lasten
        T_dev[i] = T_l[i]/(utväxling * förluster)               # Vad motorn kommer behöva utveckla
        U_motor[i] = pid.update(w_last[i-1], r_ref)             # Här är det klart fel, vår regulator skall regularisera spänning inte rotationshastighet.
        
        # Försumma att dw == 0
        # w_motor[i] = (U_motor[i] - resistans * I_motor[i]) / spänningskonstant
        # I_motor[i] = T_dev[i]/spänningskonstant
        
        # dw != 0
        # formeln för motorns varvtal:
        w_motor[i] = (spänningskonstant * dt * U_motor[i] - dt * T_dev[i] * resistans + J * resistans * w_motor[i-1]) / (J * resistans + dt * spänningskonstant**2)
        # Tar ut strömmen: 
        I_motor[i] = (J*(w_motor[i] - w_motor[i-1]) + T_dev[i])/spänningskonstant
        
        w_last[i] = w_motor[i] / (utväxling)
        v_last[i] = w_last[i] * vinschRadie
        
        P_batt[i] = I_motor[i]*U_motor[i]

        P_motor[i] = T_dev[i]*w_motor[i]
        I_batt[i] = P_batt[i]/U_batt

        if i > 0: #Eulers metod
            a_last[i] = (v_last[i] - v_last[i-1]) / dt
            s_last[i] = s_last[i-1] + dt*v_last[i]
            W_batt[i] = W_batt[i-1] + dt*P_batt[i]

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
