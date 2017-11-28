#!/usr/bin/env python
#RI_PI


import time
import serial
import RPi.GPIO as GPIO
import time
import sys
import os
import thread as checkerThread #Safety checks during movement
import thread as timeThread #thread that checks if rotations are being counted
import thread as batteryThread
import RI_config as config

currentFile=1
timeSinceLast=0 # time since last rotation
maxSpeed=100
motorA=12 #motors
motorB=10 
hallA=8 #Inturrupt
hallB=11
currentPin=7 #pulse current to keep battery active
distChange=0
dRot=0.4 #How far the ring moves with each activation of a hall effect sensor
GPIO.setwarnings(False) 
SPICLK = 13
SPIMISO = 15
SPIMOSI = 16
SPICS = 18
##### change with RI_num, max and min measured between moving part and far end, NOT between rings
maxDist=40
minDist=2
ringOffset=17
#####
forceSetpoint = 0
RI_num = 7 #determines properties of the implant (maxDist, etc.)
terminator="\n" #terminator for serial input line
moving=False
motorActive=True
led=19

def calibrate(calibrationDist): # Give RPi absolute distance by giving starting point and calculating relative distance from there
    global absDist
    absDist=calibrationDist
    updateFile()

def milToDigi(posnInMil): #conversion to digital (0-1023)
    conversionConst=float(posnInMil)/(maxDist-minDist)
    digiDist=int(round(1023*conversionConst))
    return digiDist

def digiToMil(posDigi): #conversion to mm
    conversionConst=float(posDigi/1023)
    milDist=(maxDist-minDist)*conversionConst
    return milDist

def safetyCheck(posMil): #Check boundaries at the ends of the implant
    global motorActive
    if posMil < minDist:
        motorStop()
        writeToSerial("ATTN: Rings too close together. Motors stopped.\n")
        motorActive=False
    elif posMil>maxDist:
        motorStop()
        writeToSerial("ATTN: Rings too far apart. Motors stopped.\n")
        motorActive=False


def updateFile():
    #current distance between clamps in mm
    storeFile=open("/home/pi/RI_Implant/positionStorage"+str(currentFile)+".txt", "w")
    storeFile.write(str(absDist)+"\n"+str(float(time.time()*1000))) #store absolute distance in a file in case of power failure
    storeFile.close()
    changeCurrentFile()

def moveToPosn(goalMil):#Start new threads for moving and safety checks
    checkerThread.start_new_thread(checkGoal, (goalMil,))
    time.sleep(0.05)
    timeThread.start_new_thread(checkTime,())

def checkTime():
    global lastTime
    global motorActive
    safetyTime=4000
    lastTime=float(time.time()*1000)
    while moving==True and motorActive==True:
        currentTime=float(time.time()*1000)
        if float(currentTime-lastTime)>safetyTime: #If moving and gone over 2 seconds without detecting a rotation, stop motor
            motorStop()
            motorActive=False
            writeToSerial("ATTN: Motor rotations not detected. Movement Stopped.\n")
            timeThread.exit()
        time.sleep(0.1)
    if motorActive==False:
        writeToSerial("ATTN: Motor not enabled.\n")
    timeThread.exit()

    

def checkGoal(goalMil): # Move to a position while performing safety checks
    global moving
    global motorActive
    moving=True
    try:
        if goalMil > absDist:
            pull()
            while goalMil > absDist and motorActive==True:
                safetyCheck(absDist)
                time.sleep(0.1)
        else:
            compress()
            while goalMil < absDist and motorActive==True:
                safetyCheck(absDist)
        if motorActive==False and absDist<minDist and goalMil>minDist: #allow movement out of danger zones
            motorActive=True
            pull()
            writeToSerial("ATTN: Motor enabled to move away from the end.\n")
            while goalMil > absDist and motorActive==True:
                time.sleep(0.01)
        elif motorActive==False and absDist>maxDist and goalMil<maxDist:
            motorActive=True
            compress()
            writeToSerial("ATTN: Motor enabled to move away from the end.\n")
            while goalMil < absDist and motorActive==True:
                time.sleep(0.01)
    except:
        motorStop()
        motorActive=False
        writeToSerial("ATTN: Error. Motor stopped.\n")
    motorStop()
    moving=False
    checkerThread.exit()

def writeToSerial(text):
    ser.flushOutput()
    ser.write(text)

# read SPI data from MCP3008 chip, 8 possible adc's (0 thru 7)
def readadc(adcnum, clockpin, mosipin, misopin, cspin):
        if ((adcnum > 7) or (adcnum < 0)):
                return -1
        GPIO.output(cspin, True)
 
        GPIO.output(clockpin, False)  # start clock low
        GPIO.output(cspin, False)     # bring CS low
 
        commandout = adcnum
        commandout |= 0x18  # start bit + single-ended bit
        commandout <<= 3    # we only need to send 5 bits here
        for i in range(5):
                if (commandout & 0x80):
                        GPIO.output(mosipin, True)
                else:
                        GPIO.output(mosipin, False)
                commandout <<= 1
                GPIO.output(clockpin, True)
                GPIO.output(clockpin, False)
 
        adcout = 0
        # read in one empty bit, one null bit and 10 ADC bits
        for i in range(12):
                GPIO.output(clockpin, True)
                GPIO.output(clockpin, False)
                adcout <<= 1
                if (GPIO.input(misopin)):
                        adcout |= 0x1
 
        GPIO.output(cspin, True)
        
        adcout >>= 1       # first bit is 'null' so drop it
        return adcout

def updateDistance():
    global absDist
    absDist = float(absDist) + float(distChange)

def magnetDetected(channel):
    #inturrupt function to detect hall effect activation
    if moving==True:
        updateDistance() #only update distance if moving - prevents accidental double triggers when the implant is stopped or a fluxuation in voltage occurs
    updateFile()
    global lastTime
    lastTime=float(time.time()*1000)

def magnetDetected2(channel):
    if moving==True:
        updateDistance() #only update distance if moving - prevents accidental double triggers when the implant is stopped or a fluxuation in voltage occurs
    updateFile()
    global lastTime
    lastTime=float(time.time()*1000)

# Pull rings apart?
def pull():
    if motorActive==True:
        global distChange
        distChange=dRot
        pA.start(maxSpeed)
        pB.start(0)
    else:
        motorStop()

# Pushes rings together        
def compress(): 
    if motorActive==True:
        global distChange
        distChange=-dRot
        pB.start(maxSpeed)
        pA.start(0)
    else:
        motorStop()
        
def slow(direction, spd): #move the rings at a % of maxSpeed
## 0=compress, 1=pull
    global distChange
    if direction == 0:
        distChange=-1
        pA.start(maxSpeed*spd)
        pB.start(0)
    else:
        distChange=1
        pB.start(maxSpeed*spd)
        pA.start(0)

def getIP():
    os.system("sudo ifconfig | grep inet.*Bcast.*Mask > /home/pi/RI_Implant/wlan0.txt")
    f=open("/home/pi/RI_Implant/wlan0.txt", "r")
    writeToSerial("ATTN: "+f.readline().strip(' ')+"\n")
        
def motorStop(): # Stop motors
    pA.stop()
    pB.stop()

def readForce(): # convert analog input from force sensor into digital input for RPi
    forceSensor=0
    force = readadc(forceSensor, SPICLK, SPIMOSI, SPIMISO, SPICS)     
    return force
            
def sendSerialData(): #Output serial data to laptop
    global positionSetpoint
    forceValue = readForce()
    positionValue = float(absDist) + float(ringOffset)
    voltage = 5000
    currentTime=time.time()
    timeStamp =(currentTime-startTime)*1000
    pSet=float(positionSetpoint)+float(ringOffset) #setpoint
    mSpeed=0
    outputStr=("SNSR[%010lu] %4f %4i %4f %4i %4i %4i %4i\n" % (timeStamp, float(positionValue), int(forceValue), float(pSet), int(forceSetpoint), int(mSpeed), RI_num, int(voltage)))
    print (outputStr)
    writeToSerial(outputStr) 
    
    
def initPins():# Setup GPIO in board mode
    GPIO.setmode (GPIO.BOARD)
    # set up the SPI interface pins
    GPIO.setup(SPIMOSI, GPIO.OUT)
    GPIO.setup(SPIMISO, GPIO.IN)
    GPIO.setup(SPICLK, GPIO.OUT)
    GPIO.setup(SPICS, GPIO.OUT)
    GPIO.setup(motorA, GPIO.OUT, initial=0)
    GPIO.setup(motorB, GPIO.OUT, initial=0)
    GPIO.setup(currentPin, GPIO.OUT, initial=0)
    GPIO.setup(hallA, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(hallB, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.add_event_detect(hallA, GPIO.RISING, callback=magnetDetected, bouncetime=600)
    #GPIO.add_event_detect(hallB, GPIO.RISING, callback=magnetDetected2, bouncetime=600)
    global pA
    pA=GPIO.PWM(motorA, 100)
    global pB
    pB=GPIO.PWM(motorB, 100)

def reconnect():
    updateFile()
    python=sys.executable
    ser.close()
    os.execl(python, python, * sys.argv)

def checkSerialInput(): # Parse serial inputs
    global motorActive
    global RI_num
    global maxDist
    global minDist
    global ringOffset
    flag=False
    cmd=""
    while flag==False: #read until the terminator symbol, \n, is detected
        try:
            c=ser.read(1)
            if c==terminator:
                flag=True
            else:
                cmd+=c
        except:
            motorStop()
            motorActive=False
            reconnect()
    if cmd=="g": #handshake between laptop and Rpi
        time.sleep(0.05)
        sendSerialData()
    elif cmd[0]=="P":
        try:
            dGoal=float(cmd[1:])
            global positionSetpoint
            if dGoal-ringOffset < minDist or dGoal-ringOffset > maxDist:
                writeToSerial("ATTN: Command rejected. Position setpoint too close to an end.\n")
            else:
                if abs(dGoal-ringOffset-absDist)>=dRot:
                    motorStop()
                    writeToSerial("ATTN: Changing setpoint to "+str(dGoal)+ ".\n")
                    positionSetpoint=dGoal-ringOffset
                    moveToPosn(positionSetpoint)
                else:
                    writeToSerial("ATTN: Position change below sensor resolution.\n")
        except:
            writeToSerial("ATTN: Command formatted incorrectly.\n")
        
    elif cmd[0]=="I":
        try:
            RI_num=int(cmd[1:])
            (maxDist, minDist, ringOffset) = config.riConfig(RI_num)
            writeToSerial("ATTN: Changing Implant # to " + str(RI_num) +".\n")
        except:
            writeToSerial("ATTN: Command formatted incorrectly.\n")
       
    elif cmd=="MSTRT":
        writeToSerial("ATTN: Motor activated.\n")
        motorActive=True
        if abs(absDist-positionSetpoint)>=dRot:
            moveToPosn(positionSetpoint)
    elif cmd=="MSTOP":
        motorStop()
        writeToSerial("ATTN: Motor stopped.\n")
        motorActive=False
    elif cmd[0]=="C":
        try:
            calibrationDist=float(cmd[1:])
            if calibrationDist < minDist or calibrationDist > maxDist:
                writeToSerial("ATTN: Calibration distance too close to an end. Did you make a mistake in measuring?\n")          
            else:
                calibrate(calibrationDist)
                writeToSerial("ATTN: Calibrated.\n")
        except:
            writeToSerial("ATTN: Command formatted incorrectly.\n")
    elif cmd[0]=="D":
        try:
            getIP()
        except:
            writeToSerial("ATTN: Error retrieving connection data.\n")
    elif cmd[0]=="F":
        writeToSerial("ATTN: minDist: "+str(minDist)+", maxDist: "+str(maxDist)+", ringOffset: "+str(ringOffset)+".\n")
    else:
        writeToSerial('ATTN: Command not recognized. Received @' + str(cmd) + '@\n')
    time.sleep(0.05)

def keepBatteryAwake(): # pulse over a resistor every 1.4s to keep the battery awake
    interval=1400
    lastPulse=time.time()*1000
    while True:
        if time.time()*1000-lastPulse>interval:
            GPIO.output(currentPin, 1)
            time.sleep(0.04)
            GPIO.output(currentPin, 0)
            lastPulse=time.time()

def changeCurrentFile():
    global currentFile
    if currentFile==1:
        currentFile=2
    elif currentFile==2:
        currentFile=1

initPins()
#batteryThread.start_new_thread(keepBatteryAwake, ())
serialFlag=False
while serialFlag==False: #Don't start until connection is aquired
    try:
        ser=serial.Serial('/dev/rfcomm0')
        serialFlag=True
        print ("Connected")
        ser.flushInput()
        ser.flushOutput()
    except :
        print ("Could not connect to rfcomm0")
        time.sleep(3)
ser.timeout=1
ser.baudrate=115200
startTime=time.time()
tsA=0
tsB=0
absA=0
absB=0
try:
    storeFile=open("/home/pi/RI_Implant/positionStorage"+str(currentFile)+".txt", "r") #File for position storage to give long term memory in case of power failure
    absA=float(storeFile.readline())
    tsA=float(storeFile.readline())
    storeFile.close()
except:
    print("Error reading /home/pi/RI_Implant/positionStorage"+str(currentFile)+"\n")

changeCurrentFile()
try:
    storeFile=open("/home/pi/RI_Implant/positionStorage"+str(currentFile)+".txt", "r") #File for position storage to give long term memory in case of power failure
    absB=float(storeFile.readline())
    tsB=float(storeFile.readline())
    storeFile.close()
except:
    print("Error reading /home/pi/RI_Implant/positionStorage"+str(currentFile)+"\n")
if tsA>tsB:
    absDist=absA
else:
    absDist=absB
positionSetpoint=absDist
(maxDist, minDist, ringOffset) = config.riConfig(RI_num)

while True:
    checkSerialInput()
GPIO.cleanup()

# RI_implant
