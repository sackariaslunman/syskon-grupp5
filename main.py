import numpy as np
import math
from matplotlib import pyplot as plt


#motorns variabler
resistans = 0.0459 # [R] resistans
spänningskonstant = 0.0184
utväxling = 3.5**5 # 1 : 525.2
förluster = 0.95**5
max_u = 10.67 # [v] motorns max spänning, volt
max_i = 28.8 # [A] motorns max ström, ampere
spärr = bool # spärr för när systemet ska låsas fast

#Ytterliggare variabler
MLast = 1500
g = 9.82 #gravitation
vajer_dist = 8. # [m] meter

#Uträknade siffror
tyngdkraften = MLast * g * math.sin(12*2*math.pi/360) #vertikaltlyft
cr = 0.05 # Friktionskonstanten
friktionskraften = cr * MLast * g * math.cos(12*2*math.pi/360) #normalkraften

#Inparametrar

dT = 0.1 #[s], sekunder per tidssteg för simulering
T = 500.0 # [s], sekunder för hela simuleringen
N = round (T/dT) #antal steg att simulera avrundat till heltal

v_last = np.zeros(N) #hastighet på vinsch - begynelsevärde
a_last = np.zeros(N) #acceleration på vinsch - begynelsevärd
s_last = np.zeros(N) #sträckan på last - begynelsevärde


F_last = np.zeros(N) # [N] lastens kraft, Newton
w_last = np.zeros(N) # [rad/s] lastens rotationshastighet
w_motor = np.zeros(N) # [rad/s] motorns ratationshastighet
T_l = np.zeros(N) # [Nm] lastens vridmoment, Newton meter
T_dev = np.zeros(N) # [Nm] motorns vridmomentn Newton meter
P_motor = np.zeros(N) # [W] motorns effekt, watt
I_motor = np.zeros(N) # [A] motorns ström, ampere
U_motor = np.zeros(N) # [V] motorns spänning, volt
U_a_motor = np.zeros(N) # [V/s^2] motorns accelaration, volt
P_batt = np.zeros(N) # [W] batteriets effekt
I_batt = np.zeros(N) # [A] batteriets ström, ampere
U_batt = 12 # [V] batteriets spänning, volt - konstant
W_batt = np.zeros(N) # [W] totala förbrukning i batteriets watt

t = np.arange(0., 500., 0.1)

def main():
    v_last_ref = 0.0366
    for i in range(N):
        vinschRadie = 0.05*(vajer_dist/(vajer_dist + (s_last[i]*(1/3)))) 
        # Vinschradie går från 5 cm (när vajern är indragen) till 3.75 cm (när vajern är fullt utdragen)

        accelerationskraften = a_last[i]*MLast #accelerations kraften utanför för det är samma i alla lägen
        
        if(s_last[i] < 6 and v_last_ref > 0): # Båten är på trailern och åker ner mot vattnet
            v_last_ref = 0.0366
            F_last[i] =(tyngdkraften - friktionskraften + accelerationskraften)

        elif(s_last[i] >= 6 and s_last[i] <= 8 and v_last_ref >0): # Båten åker ut i vattnet i 2 meter
            v_last_ref = 0.0366
            F_last[i] =(accelerationskraften/math.cos(12))

        elif((s_last[i] > 8 and v_last_ref > 0) or (s_last[i] > 6 and v_last_ref < 0)): # Båten vänder riktning i vattnet och åker mot trailer
            v_last_ref = -0.0366
            F_last[i] =(accelerationskraften/math.cos(12))

        else: #Båten är på trailern och dras upp
            F_last[i] =(tyngdkraften + friktionskraften + accelerationskraften)

            if s_last[i] < 0.1: # om det är 10 cm kvar på så bromsar den in
                v_last_ref = 0
            else: # annars så drar den upp båten som vanligt
                v_last_ref = -0.0366
                

        T_l[i] =F_last[i] * vinschRadie 
        T_dev[i] =(T_l[i]/(utväxling * förluster))
        w_last[i] =(v_last[i] /(vinschRadie))
        w_motor[i] =(w_last[i]*utväxling)
        P_motor[i] =(T_dev[i]*w_motor[i])
        I_motor[i] =(T_dev[i]/spänningskonstant)
        U_motor[i] =(w_motor[i]*spänningskonstant + resistans*I_motor[i])
        P_batt[i] =(I_motor[i]*U_motor[i])
        I_batt[i] =(P_batt[i]/U_batt)
        if i < N-1: #Eulers funktion
            a_last[i+1] =((1/2)*(v_last_ref - v_last[i])) # beräkna den nya accelerationen
            v_last[i+1] =(v_last[i] + dT*a_last[i]) # beräkna den nya hastigheten
            s_last[i+1] =(s_last[i] + dT*v_last[i]) # beräkna den nya sträckan
            W_batt[i+1] =(W_batt[i] + dT*P_batt[i]) # beräkna den nya effekten
        

def regler_spänning():
    u_last_ref = max_u
    #s_last[0] = 0
    spärr = False
    tryck = 0 # tryck sensor
    for i in range(N):
        if spärr == True: #om spärren är påslagen så stoppar systemet.
            I_motor[i] = 0
            F_last[i] = F_last [i-1]
            w_last[i] = 0
            v_last[i] = 0
            a_last[i] = 0
            w_motor[i] = 0
            s_last[i] = s_last[i-1]
            U_motor[i] = 0
            T_l[i] = 0
            T_dev[i] = 0
        else: #annars så fortsätter systemet som vanligt
            accelerationskraften = a_last[i]*MLast #accelerations kraften utanför för det är samma i alla lägen
            vinschRadie = 0.05*(vajer_dist/(vajer_dist + (s_last[i]*(1/3)))) 
            # Vinschradie går från 5 cm (när vajern är indragen) till 3.75 cm (när vajern är fullt utdragen)

            if(s_last[i] < 6 and u_last_ref > 0): # Båten är på trailern och åker ner mot vattnet
                u_last_ref = max_u
                F_last[i] =(tyngdkraften - friktionskraften + accelerationskraften)

            elif(s_last[i] >= 6 and s_last[i] <= 8 and u_last_ref >0): # Båten åker ut i vattnet i 2 meter
                u_last_ref = max_u
                F_last[i] =(accelerationskraften/math.cos(12))

            elif((s_last[i] > 8 and u_last_ref > 0) or (s_last[i] > 6 and u_last_ref < 0)): # Båten vänder riktning i vattnet och åker mot trailer
                u_last_ref = -max_u
                F_last[i] =(accelerationskraften/math.cos(12))

            else: #Båten är på trailern och dras upp
                F_last[i] =(tyngdkraften + friktionskraften + accelerationskraften)

                if s_last[i] < 0.1: # om det är 10 cm kvar på så bromsar den in
                    u_last_ref = 0
                    if U_motor[i] < 0.01:
                        spärr = True
                else: # annars så drar den upp båten som vanligt
                    u_last_ref = -max_u

            w_motor[i] = (U_motor[i] - resistans*I_motor[i])/spänningskonstant
            w_last[i] = w_motor[i]/utväxling
            v_last[i] = w_last[i]*vinschRadie
            T_l[i] = F_last[i]*vinschRadie
            T_dev[i] = (T_l[i]/(utväxling * förluster))
            if T_dev[i]/spänningskonstant > max_i: 
                # Om systemet vill få mer ström än vad elmotorn klarar av så stängs systemet av
                spärr = True
            I_motor[i] = T_dev[i]/spänningskonstant

            if i < N-1:
                U_a_motor[i+1] = (1/2)*(u_last_ref-U_motor[i])
                U_motor[i+1] =(U_motor[i] + dT*U_a_motor[i])
                if(i>0):
                    a_last[i] = (v_last[i]-v_last[i-1])
                s_last[i+1] = (s_last[i] + dT*v_last[i])
            
            if(tryck > 100): #Våran trycksensor ska stänga av systemet
                print("tryck sensor")
                spärr = True

if __name__ == "__main__":
    #main()
    regler_spänning()
    
    plot1 =  plt.figure(1) # plot av hastigheten beroende på tiden
    plt.plot(t, v_last)
    plt.xlabel('tid')
    plt.ylabel('hastighet [m/s]')

    plot2 = plt.figure(2) # plot av sträckan beroende på tiden
    plt.plot(t, s_last)
    plt.xlabel('tid')
    plt.ylabel('distans [m]')
    
    plot3 = plt.figure(3) # plot av accelerationen beroender på tiden
    plt.plot(t, a_last)
    plt.xlabel('tid')
    plt.ylabel('acceleration [m/s^2]')
    
    plot4 = plt.figure(4) # plot av strömmen i motorn beroende på tiden
    plt.plot(t, I_motor)
    plt.xlabel('tid')
    plt.ylabel('ström [A]')

    plot5 = plt.figure(5) # plot av strömmen i motorn beroende på tiden
    plt.plot(t, U_motor)
    plt.xlabel('tid')
    plt.ylabel('spänning [V]')
    
    plt.show()

