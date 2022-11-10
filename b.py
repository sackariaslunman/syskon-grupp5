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
vinschRadie = 0.05 # [m] meter

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

dT = 0.01 #[s], sekunder per tidssteg för simulering
T = 500.0 # [s], sekunder för hela simuleringen
N = round (T/dT) #antal steg att simulera avrundat till heltal

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

t = np.arange(0., 500., dT)

F_last =(tyngdkraften - friktionskraften)

# Oklart vad J ska vara
J = 0.1

def main():
    F_last =(tyngdkraften - friktionskraften) * 1

    for i in range(N):
    
        T_l[i] =F_last * vinschRadie
        T_dev[i] = T_l[i]/(utväxling * förluster)

        U_motor[i] = U_batt * 0.5
        
        # Försumma att dw == 0
        # w_motor[i] = (U_motor[i] - resistans * I_motor[i]) / spänningskonstant
        # I_motor[i] = T_dev[i]/spänningskonstant
        
        # dw != 0
        w_motor[i] = (spänningskonstant * dT * U_motor[i] - dT * T_dev[i] * resistans + J * resistans * w_motor[i-1]) / (J * resistans + dT * spänningskonstant**2)
        I_motor[i] = (J*(w_motor[i] - w_motor[i-1]) + T_dev[i])/spänningskonstant
        
        w_last[i] = w_motor[i] / (utväxling * förluster)
        v_last[i] = w_last[i] * vinschRadie
        
        P_batt[i] = I_motor[i]*U_motor[i]
        P_motor[i] = T_dev[i]*w_motor[i]
        I_batt[i] = P_batt[i]/U_batt

        if i > 0: #Eulers metod
            a_last[i] = (v_last[i] - v_last[i-1]) / dT
            s_last[i] = s_last[i-1] + dT*v_last[i]
            W_batt[i] = W_batt[i-1] + dT*P_batt[i]

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
