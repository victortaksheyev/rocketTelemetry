import serial
import numpy as np
import matplotlib.pyplot as plt
from drawnow import *

plt.style.use('dark_background')

#arrays for all data
aXA = []
aYA = []
aZA = []
gyroXA = []
gyroYA = []
gyroZA = []
humidityA = []
pressureA = []
altitudeA = []
temperatureA = []

#connecting to input device and baud [change if not using my laptop]
arduinoData = serial.Serial('/dev/cu.usbmodem14101', 115200) 
#interactive mode
plt.ion()

thetaI = (np.pi/2)
count = 0

# GRAPH height v time ---------------------------------------------------------
def graphAlt():
    plt.plot(altitudeA, 'orange')
    plt.title('Altitude')
    plt.ylim(0, 15000)
    plt.xlabel('time (s)')
    plt.ylabel('height (ft)')
    plt.grid(b = True, axis='y', color='grey', linewidth = 0.5)
    
# GRAPH pressure v elevation --------------------------------------------------
def graphPressure():
    plt.title('Pressure v Elevation')
    plt.plot(altitudeA,pressureA, 'c')
    plt.yticks(np.arange(0, 140, 20))
    plt.xlim(0, 15000)
    plt.xlabel('Elevation (ft)')
    plt.ylabel('Presure (kPa)')
    plt.grid(b = True, axis='y', color='grey', linewidth = 0.5)
    

# GRAPH temperature v time and humidity v time (subplots) ---------------------
def graphTempHum():
    # 2 rows, 1 col, index 1 (index starts at 1 in upper left corner)
    ax1 = plt.subplot(211) 
    plt.title('temperature and humidity')
    ax2 = plt.subplot(212, sharex = ax1)
    ax1.plot(temperatureA)
    ax2.plot(humidityA)
    plt.subplots_adjust(hspace =.5)
    ax1.set_xlabel('time (s)')
    ax1.set_ylabel('temp (F)')
    ax2.set_xlabel('time (s)')
    ax2.set_ylabel('humidity (%)')
    
# GRAPH acceleration v axis (bar) ---------------------------------------------
def barAccel():
    objects = ('X', 'Y', 'Z')
    y_pos = np.arange(len(objects))
    plt.bar(y_pos, aAccel, alpha=1, color = 'orange')
    plt.ylim(-10, 10)
    plt.xticks(y_pos, objects)
    plt.ylabel('acceleration (g)')
    plt.xlabel('axis of acceleration')
    plt.title('Rocket\'s acceleration')
    plt.grid(b = True, axis='y', color='grey', linewidth = 0.5)

# GRAPH gyro (polar) ----------------------------------------------------------
def graphGyro():
    global thetaI, theta, count
    if count == 0:
        #initial theta (rocket is upright)
        theta = thetaI
    else:
        if gyroZ>0:
            theta -= gyroZ/600
        else:
             # subtracting negative to make it positive 
            theta -= gyroZ/600


    radii = (10  * (gyroY/1000)) # some number such that 10 is the max
    width = np.pi / 4

    ax = plt.subplot(111, projection='polar')
    ax.bar(theta, radii, width=width, bottom=0.0, color=(244/256, (1/(((abs(radii))*0.5)+1)), 66/256, 1))
    
    plt.grid(b = True, color='grey', linewidth = 0.5)
    plt.title('Gyroscopic Direction')
    ax.set_ylim(0,5)
    
    

while True: #loops forever
    # handling no data case ---------------------------------------------------
    while (arduinoData.inWaiting()== 0):
        pass

    arduinoString = arduinoData.readline() 
    
    # in py3 need to convert
    arduinoString = arduinoString.decode('utf_8')
    dataArray = arduinoString.split(',')
    
    # storing array entries split at , to respective variables
    aX = float(dataArray[0])
    aY = float(dataArray[1])
    aZ = float(dataArray[2]) 
    gyroX = float(dataArray[3])
    gyroY = float(dataArray[4])
    gyroZ = float(dataArray[5])
    humidity = float(dataArray[6])
    pressure = float(dataArray[7])
    altitude = float(dataArray[8])
    temperature = float(dataArray[9])
    aAccel = aX, aY, aZ    
    
    #conversions --------------------------------------------------------------
    
    #converting PA to kPA
    pressureR = round((pressure/1000),2)
        
    # populate arrays ---------------------------------------------------------
    aXA.append(aX)
    aYA.append(aY)   
    aZA.append(aZ)
    gyroXA.append(gyroX)   
    gyroYA.append(gyroY)
    gyroZA.append(gyroZ)   
    humidityA.append(humidity)
    pressureA.append(pressureR)   
    altitudeA.append(altitude)
    temperatureA.append(temperature)
    
    
    # plotting & displaying arrays --------------------------------------------
    plt.figure(1)
    drawnow(graphAlt)
  
    plt.figure(2)
    drawnow(graphTempHum)
    
    plt.figure(3)
    drawnow(barAccel)
    
    plt.figure(4)
    drawnow(graphPressure)
    
    plt.figure(5)
    drawnow(graphGyro)
   
    # limiting length of array and graph --------------------------------------
    if(count>=0):
        aXA.pop(0)
        
    if (count>20):
        temperatureA.pop(0)

    count = count+1
    
    #prevents crash -----------------------------------------------------------
    plt.pause(.001) 