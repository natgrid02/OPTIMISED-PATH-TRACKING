import time
from MPU9250 import MPU9250
import RPi.GPIO as gpio
import math

if __name__ == '__main__':
    lat1 = math.radians(12.970060) #Current lat point converted to radians
    lon1 = math.radians(79.170196) #Current long point converted to radians
    R = 6378.1 #Radius of the Earth
    
    mpu9250 = MPU9250()
    cu=0
    pgz=0
    drc=360
    gpio.setmode(gpio.BCM)
    gpio.setwarnings(False)

    hallpin = 18
    gpio.setup(hallpin, gpio.IN)

    prv=1
    
    while True:
      if(cu>2):
        gyro = mpu9250.readGyro()
        gz = int(gyro['z'])
        gz=gz-pgz
        #print(gz)
        #print(gpio.input(hallpin))
        hs=int(gpio.input(hallpin))
        if(prv!=hs):
            #print("state change")
            if(hs==0):
             #  print("hit")
               if(gz>0):
                  drc=drc-gz
                  if(drc<0):
                      drc=359
               elif (gz<0):
                  gz=-(gz)
                  drc=drc+gz
                  if(drc>360):
                      drc=1
               print(drc)
               bbr=int(drc)

               brng=math.radians(bbr)
               d = 0.000914
   
               lat2 = math.asin( math.sin(lat1)*math.cos(d/R) +
                    math.cos(lat1)*math.sin(d/R)*math.cos(brng))

               lon2 = lon1 + math.atan2(math.sin(brng)*math.sin(d/R)*math.cos(lat1),
                    math.cos(d/R)-math.sin(lat1)*math.sin(lat2))

               lat2 = math.degrees(lat2)
               lon2 = math.degrees(lon2)

               print("{lat: "+str(lat2)+", lng: "+str(lon2)+"},")

               lat1=math.radians(lat2)
               lon1=math.radians(lon2)
             
            prv=hs

      else:
          cu=cu+1
          gyro = mpu9250.readGyro()
          #print(" gz = " , ( gyro['z'] ))
          pgz = int(gyro['z'])

      time.sleep(0.3)

