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

dT = 0.1 #[s], sekunder per tidssteg för simulering
T = 40.0 # [s], sekunder för hela simuleringen
N = round (T/dT) #antal steg att simulera avrundat till heltal

v_last = np.zeros(N) #hastighet på vinsch - begynelsevärde
a_last = np.zeros(N) #acceleration på vinsch - begynelsevärd
s_last = np.zeros(N) #sträckan på last - begynelsevärde


F_last = np.zeros(N)
w_last = np.zeros(N)
w_motor = np.zeros(N)
T_l = np.zeros(N)
T_dev = np.zeros(N)
w_last = np.zeros(N)
w_motor = np.zeros(N)
P_motor = np.zeros(N)
I_motor = np.zeros(N)
U_motor = np.zeros(N)
P_batt = np.zeros(N)
I_batt = np.zeros(N)
U_batt = 12 # [v] Volt - konstant
W_batt = np.zeros(N)

t = np.arange(0., 40., 0.1)
def main():
    for i in range(N):
        accelerationskraften = a_last[i]*MLast
        if(i < 150): # Först åker båten ut i 15 sekunder på trailern
            w_last_ref = 0.733
            F_last[i] =(tyngdkraften - friktionskraften + accelerationskraften)
        elif(i >= 150 and i < 200): # Båten fortsätter åka ut fast i 5 sekunder fast nu i vatten
            w_last_ref = 0.733
            F_last[i] =(accelerationskraften/math.cos(12))
        elif(i >= 200 and i < 250): # Båten vänder riktning i vattnet ock åket tillbaka i 5 sekunder
            w_last_ref = -0.733
            F_last[i] =(accelerationskraften/math.cos(12))
        else: #Båten dras upp på trailer i 15 sekunder
            w_last_ref = -0.733
            F_last[i] =(tyngdkraften + friktionskraften + accelerationskraften)
        T_l[i] =F_last[i] * vinschRadie
        T_dev[i] =(T_l[i]/(utväxling * förluster))
        w_last[i] =(v_last[i] /(vinschRadie))
        w_motor[i] =(w_last[i]*utväxling)
        P_motor[i] =(T_dev[i]*w_motor[i])
        I_motor[i] =(T_dev[i]/spänningskonstant)
        U_motor[i] =(w_motor[i]*spänningskonstant + resistans*I_motor[i])
        P_batt[i] =(I_motor[i]*U_motor[i])
        I_batt[i] =(P_batt[i]/U_batt)
        if i < N-1:
            a_last[i+1] =((1/4)*(w_last_ref - w_last[i]))
            v_last[i+1] =(v_last[i] + dT*a_last[i])
            s_last[i+1] =(s_last[i] + dT*v_last[i])
            W_batt[i+1] =(W_batt[i] + dT*P_batt[i])

    plot1 =  plt.figure(1)
    plt.plot(t, v_last)
    plt.xlabel('tid')
    plt.ylabel('hastighet [m/s]')

    plot2 = plt.figure(2)
    plt.plot(t, s_last)
    plt.xlabel('tid')
    plt.ylabel('distans [m]')
    
    plot3 = plt.figure(3)
    plt.plot(t, a_last)
    plt.xlabel('tid')
    plt.ylabel('acceleration [m/s^2]')

    
    plot4 = plt.figure(4)
    plt.plot(t, I_motor)
    plt.xlabel('tid')
    plt.ylabel('ström [A]')
    
    plt.show()
if __name__ == "__main__":
    main()