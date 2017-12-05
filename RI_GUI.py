#!/usr/bin/python
import RI_functions as RIFunc

import serial
import matplotlib.pyplot as plt
import multiprocessing
from multiprocessing import Lock, Value
import glob
import warnings

warnings.filterwarnings('ignore', category=UserWarning)

xAxisWidth=120000
yPosMin=0
yPosMax=70
pointsPerPlot=300

connStatus=False
RUN_PROMPT=True

def updateGraphics(lock, liveStream_ref, plotGraphs_ref, serial_ref, ri_num_ref):
    """Refreshes data stream"""

    fig=plt.figure();
    axPos=fig.add_subplot(2,1,1)
    
    axPos.set_ylim(yPosMin,yPosMax)
    
    axFrc=fig.add_subplot(2,1,2)
    
    axFrc.set_ylim(0,1024)
    plt.ion()
    n=0
    pointList=[None]*(pointsPerPlot+3)
    forceList=[None]*(pointsPerPlot+3)
    setPtList=[None]*(pointsPerPlot+3)
    while (liveStream_ref.value == 1):
        #Read sensor values
        lock.acquire()
        sensorValues=RIFunc.parseSerialData(serial_ref)  
        lock.release()
        timeStamp=sensorValues[0]
        snsrPosition=sensorValues[1]
        snsrForce=sensorValues[2]
        stptPosition=sensorValues[3]
        lock.acquire()
        positionMM=float(snsrPosition)
        lock.release()
        if (plotGraphs_ref.value == 1):
            while n>(pointsPerPlot):
                lock.acquire()
               # print (pointList)
                l,=pointList[0]
                pointList[0:len(pointList)-1]=pointList[1:]
                l.remove()
                l,=forceList[0]
                forceList[0:len(forceList)-1]=forceList[1:]
                l.remove()
                l,=setPtList[0]
                setPtList[0:len(setPtList)-1]=setPtList[1:]
                l.remove()
                del l
                n-=1
                lock.release()
            lock.acquire()
            axPos.set_xlim(timeStamp-xAxisWidth,timeStamp)
            setPtList[n] = (axPos.plot(timeStamp, stptPosition, 'ro'))
            pointList[n] = (axPos.plot(timeStamp, positionMM , 'bo'))
            axFrc.set_xlim(timeStamp-xAxisWidth,timeStamp)
            forceNewtons=RIFunc.riConvert(int(snsrForce), 'dig2N', ri_num_ref.value)
            forceList [n]=(axFrc.plot(timeStamp,snsrForce, 'bo'))
            n+=1
          #  print (n)
          #  axFrc.plot(timeStamp,forceNewtons, 'bo')
            lock.release()
            
            
            
        lock.acquire()
        axPos.set_title("Position Sensor Output (mm)   [%0.2f mm]" % (positionMM) )
        #axFrc.set_title("Force Sensor Output (N)   [%0.2f dig]" % (forceNewtons) )
        axFrc.set_title("Force Sensor Output (N)   [%0.2f dig, %0.2f N]" % (int(snsrForce), forceNewtons ) )
        lock.release()
        plt.pause(0.03)




##########
## MAIN ##
##########
if __name__ == '__main__':
    LIVE_STREAM=Value('d',1)
    PLOT_GRAPHS=Value('d',1)
    ri_num=Value('i',7);
    lock = Lock()

    while (RUN_PROMPT):
        
        if (connStatus):
            prompt="{{connected RI"+str(ri_num.value)+" on "+ser.port+"}}"
        else:
            prompt="{{disconnected}}"
        
        userInput=raw_input(prompt+" [help/connect/disconnect/quit/i/p/pid/plot/mstart/mstop/calibrate/ip/info]--> ").strip()
        #check for empty string
        if (userInput == "q") or (userInput == "quit"): #Quit
            lock.acquire()
            LIVE_STREAM.value = 0
            lock.release()
            RUN_PROMPT=False
        elif (userInput == "h") or (userInput == "help"): #Quit
            print """\n\nRI Controller & Live Data Stream \n
 h/help       - display help menu
 c/connect    - Connect to serial
 d/disconnect   - disconnect from serial
 q/quit       - Quit RI\n
 plot         - Toggle Plotting
 
 RI Commands:
 i      - Change implant #
 p      - Change position setpoint
 pid    - Change PID values
 plot   - Toggles plotting
 mstart - Enables motors
 mstop  - Disables motors
 calibrate  - calibrate distance
 ip     - retrieve connection data
 info   - retrieve information about the implant"""
        elif (userInput == "p"): #Position Command
            if (connStatus):
                cmd=raw_input(prompt+ " Input new setpoint (mm between rings): ")
                if not cmd:
                    print "Sending no cmd"
               # elif (float(cmd) < yPosMax) and (float(cmd) > yPosMin):
                else:
                    lock.acquire()
                    RIFunc.send2board(ser, "P"+ str(cmd))
                    lock.release()
            else:
                print("Not connected to bluetooth!")
                
        elif (userInput == "i"): #RI number cmd
            if (connStatus):
                cmd=raw_input(prompt+ " Input ri_num or 0 to cancel: ")
                if (float(cmd) != 0):
                    lock.acquire()
                    RIFunc.send2board(ser, "I"+ str(int(cmd)))
                    ri_num.value=int(cmd)
                    lock.release()
                else:
                    print "Sending no cmd"
            else:
                print("Not connected to bluetooth!")
                
        elif (userInput == "connect" or userInput == "c"):
            if not connStatus:       
                print("Available serial ports:")
                portsList=glob.glob('/dev/rfcomm*')
                print portsList

                cmd=raw_input(prompt+ " Input which # serial device /dev/rfcomm")
                #cmd=cmd.strip()
                if (len(cmd) != 1) and (len(cmd) != 2):
                    #Select last port ### in order to auto-connect to most recent port.
                    lastPort=portsList[0]
                    lastPortNum=lastPort[len(lastPort)-1]                    
                    cmd=lastPortNum
                    
                print ("Connecting to /dev/rfcomm"+str(cmd))

                try: 
                    ser = serial.Serial('/dev/rfcomm'+str(cmd))  # open serial port
                    #ser = serial.Serial('/dev/rfcomm0')  # open serial port
                
                    lock.acquire()
                    LIVE_STREAM.value = 1
                    PLOT_GRAPHS.value = 1
                    lock.release()
                    ser.baudrate=115200
                    p=multiprocessing.Process(target=updateGraphics, args=(lock, LIVE_STREAM,PLOT_GRAPHS,ser,ri_num, ))
                    p.daemon = True
                    p.start()#Start live streaming
                    connStatus=True
                    ser.flushOutput()
                    ser.flushInput()
                except serial.serialutil.SerialException:
                    print ("Error: Failed to connect! did you make a typo? Check that your serial port is available.")
     
            else:
                print ("Already connected to ")
                print ser.port
             
        elif (userInput == "disconnect" or userInput == "d"):
            lock.acquire()
            LIVE_STREAM.value = 0
            ser.close()
            lock.release()
            connStatus=False
            
        elif (userInput == "plot"):
            lock.acquire()
            if (PLOT_GRAPHS.value == 1):
                PLOT_GRAPHS.value = 0
                print("Disabling plots\n")
            else:
                PLOT_GRAPHS.value = 1
                print("Enabling plots\n")
            lock.release()
#            
#        elif (userInput == "clear"): # Clear Plots
#            lock.acquire()
#            axPos.cla()
#            axFrc.cla()
#            lock.release()
            
        elif (userInput == "pid"): #RI number cmd
            if (connStatus):
                P=float(raw_input(prompt+ " Input P value: "))
                I=float(raw_input(prompt+ " Input I value: "))
                D=float(raw_input(prompt+ " Input D value: "))
    
#                if(len(cmd)==1):
                lock.acquire()
                # Send cmd to serial port:
                #eg: XP006.000I001.000D000.000
                RIFunc.send2board(ser, "XP%07.3fI%07.3fD%07.3f" % (P,I,D) )
                lock.release()
#                else:
#                    print "Sending no cmd"
            else:
                print("Not connected to bluetooth!")
                
                
        elif (userInput == "mstart"):
            lock.acquire()
            RIFunc.send2board(ser, "MSTRT")
            lock.release()
            
        elif (userInput == "mstop"):
            lock.acquire()
            RIFunc.send2board(ser, "MSTOP")
            lock.release()
        elif (userInput == "calibrate"):
            distance=float(raw_input("Enter the current distance (mm between mobile piece and end of implant):"))
            lock.acquire()
            RIFunc.send2board(ser, "C"+str(distance))
            lock.release()
        elif (userInput == "ip"):
            lock.acquire()
            RIFunc.send2board(ser, "D")
            lock.release()
        elif (userInput == "info"):
            lock.acquire()
            lock.release()
            
