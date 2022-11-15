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
spärr = bool # det som bestämmer om systemet skall stanna eller ej. Bestäms genom trycksensor. 

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
v_ref = 0.0366

t = np.arange(0., 500., dt)


# Oklart vad J ska vara
J = 0.1

from pid import PID

def main():
    spärr = False
    #F_last =(tyngdkraften - friktionskraften) * 1       # faktorn representerar 100 %. 
    pid = PID(0.9, 5, 0.0001 , dt, (maxWmotor/utväxling), -(maxWmotor/utväxling))        # skapar regulatorn med konstanterna Kp = 0.005, Ki = 0.0002, Kd = 0.001 och max min
                                                                # spänning är det batteriet kan förse som högst. 
    tryck = 0
    v_ref = 0.0366
    for i in range(N):
        accelerationskraften = a_last[i]*MLast #accelerations kraften utanför för det är samma i alla lägen
        vinschRadie = 0.05*(vajer_dist/(vajer_dist + (s_last[i-1]*(1/3)))) # Funktion för vinschradiens förhållande till båtens position. (Förklaras i 'a' i rapport)
        if(s_last[i-1] < 6 and v_ref > 0): # Båten är på trailern och åker ner mot vattnet
            v_ref = 2*0.0366
            F_last[i] =(tyngdkraften - friktionskraften + accelerationskraften)

        elif(s_last[i-1] >= 6 and s_last[i-1] <= 8 and v_ref >0): # Båten åker ut i vattnet i 2 meter
            v_ref= 2*0.0366
            F_last[i] =(accelerationskraften/math.cos(12))

        elif((s_last[i-1] > 8 and v_ref > 0) or (s_last[i-1] > 6 and v_ref < 0)): # Båten vänder riktning i vattnet och åker mot trailer
            
            v_ref = -2*0.0366
            F_last[i] =(accelerationskraften/math.cos(12))

        else: #Båten är på trailern och dras upp
            F_last[i] =(tyngdkraften + friktionskraften + accelerationskraften)

            if tryck > 100 or s_last[i-1] < 0.1: # [Pa] om det är 10 cm kvar på så bromsar den in
                v_ref = 0
                I_motor[i] = 0

            else: # annars så drar den upp båten som vanligt
                v_ref = -2*0.0366
            

        if spärr == True: # om spärren är på
            v_ref = 0
            I_motor[i] = 0


        T_l[i] = F_last[i]*vinschRadie
        T_dev[i] = (T_l[i]/(utväxling * förluster))

        if spärr == False:
            I_motor[i] = T_dev[i]/spänningskonstant
        if T_dev[i]/spänningskonstant > maxI: 
            # Om systemet vill få mer ström än vad elmotorn klarar av så stängs systemet av
            spärr = True
        

        w_motor[i] = (U_motor[i] - resistans*I_motor[i])/spänningskonstant
        w_last[i] = w_motor[i]/utväxling
        v_last[i] = w_last[i]*vinschRadie

        
        P_batt[i] = I_motor[i]*U_motor[i]

        P_motor[i] = T_dev[i]*w_motor[i]
        I_batt[i] = P_batt[i]/U_batt
        if i < N-1:
            U_motor[i+1] = ((pid.update(v_last[i-1], v_ref)/vinschRadie)*utväxling)*spänningskonstant + resistans*I_motor[i] # Får ut fel värde just nu, vi vill få spänning men får v_ref istället
            if U_motor[i+1] > maxU:
                U_motor[i+1] = maxU
            elif U_motor[i+1] < -maxU:
                U_motor[i+1] = -maxU

        if i > 0: #Eulers metod
            a_last[i] = (v_last[i] - v_last[i-1]) / dt
            s_last[i] = s_last[i-1] + dt*v_last[i]
            W_batt[i] = W_batt[i-1] + dt*P_batt[i]

        # if(tryck > 100): #Våran trycksensor ska stänga av systemet
        #         print("tryck sensor")
        #         spärr = True

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
