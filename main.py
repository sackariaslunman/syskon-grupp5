import math


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
maxTLast = 189.88 # [Nm]
minWVinsch = 0.733 # [w] rad/s
distance = 6.6 # [m] meter
maxTime = 180 #[s] sekunder

#Uträknade siffror
TDev = maxTLast/(utväxling * förluster) #0.4672 Nm
wMotor = minWVinsch * utväxling # [w] 385 rad/s
tyngdkraften = MLast * g * math.sin(12) #vertikaltlyft
cr = 0.05 # Friktionskonstanten
friktionskraften = cr * MLast * g * math.cos(12) #normalkraften
a = 0 #acceleration
accelerationskraften = a*MLast
vLast = distance*maxTime


#modell av lasten
FLast = tyngdkraften + friktionskraften + accelerationskraften
PLast = vLast*FLast

#Inparametrar

dT = 0.1 #[s], sekunder per tidssteg för simulering
T = 40 # [s], sekunder för hela simuleringen
N = round (T/dT) #antal steg att simulera avrundat till heltal

def main():
    print("Hello pub")

if __name__ == "__main__":
    main()