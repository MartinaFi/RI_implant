import time
import datetime

notedir="/home/implant_user/Dropbox/Robot Implants/Data/notelogs"
datadir="/home/implant_user/Dropbox/Robot Implants/Data/datalogs"
def riConfig(ri_num):
#function [dig2mm dig2N zeroP zeroF threshF] = ri_config(ri_num)
#% dig2mm etc... conversion factor from digital to mm
#% zero P is the digital zero offset for position.
#%threshF is the highest value which == 0; ie: what the sensor reads when no
#%load is applied.
#
#% array of config vals in same order as output:
#% dig2mm dig2N zeroP zeroF zeroThresh
    config_vals = [ [661/20,                 (277-60)/1.4715,            -665.05,    -8,                 23], #% These values were used for Danita IV encaspsulation: 661/20  253/1.962*0+300/1.962   -665.05    -16          24;
                    [(700-25)/(50-25),       289/1.962*0+300/1.962,      -650,       -36+0*40+17.35,     36],
        #%(701-219)/(38.5-22.5)  (225-59)/(2.943-1.962)   -460    -270          55;#%old board values I3     
                    [(866-242)/(47.5-23.5),  (329-136)/(2.943-1.962),    -365,       -248,               44], # %new board values implant 3    
                    [(755-364.5)/(41.5-26),  (102.5-31.5)/(0.981-0.491), -295,       -41,                21], #%new board values implant 4     
                    [0, 198.81,  0,  0, 7],
                    [0, 198.81,  0,  0, 23],
                    [0, 198.81,  0,  0, 7]]                    
#%new board vaues     (927-249)/(44.87-21.5) (185-56)/0.981 -380 -66 21;

    dig2mm = config_vals[ri_num-1][0];
    dig2N =  config_vals[ri_num-1][1];
    zeroP =  config_vals[ri_num-1][2];
    zeroF =  config_vals[ri_num-1][3];
    threshF= config_vals[ri_num-1][4];

        
    return dig2mm, dig2N, zeroP, zeroF, threshF

def riConvert ( inputVal, conversionType, ri_num ):
#%CONVERT inputVal = value to convert. conversionType = 'digN','digMM','N','mm'
#%  converts inputVal to units of conversionType. 
#% IE: digN is equivalent to dig2Newtons()
#% if conversionType = digMM, assumption is that you're passing a mm value.
#% dig stands for digital.
    if ri_num != 1 and ri_num != 2 and ri_num!=3 and ri_num!=4 and ri_num!=5 and ri_num!=6 and ri_num!=7:
        ri_num = 1
    
    (dig2mm, dig2N, zeroP, zeroF, threshF) = riConfig(ri_num);
    
    ringSizeFactor = 1 #15/8 #change nominator based on ring size  
    
    if conversionType=='N2dig':# output DIG
        convertedVal = (inputVal*ringSizeFactor)*dig2N  + zeroF;
    elif conversionType=='mm2dig': # output DIG
        convertedVal = inputVal*dig2mm + zeroP;
    elif conversionType== 'RELmm2dig': # output DIG
        convertedVal = inputVal*dig2mm;
    elif conversionType=='dig2N': # output newtons
        if (inputVal < threshF):
            convertedVal = 0;
        else:
            convertedVal = (inputVal-zeroF)/dig2N;
            
        convertedVal = convertedVal/ringSizeFactor;
        
    elif conversionType == 'dig2mm': #% output mm
        convertedVal = (inputVal-zeroP)/dig2mm;
    else:
        convertedVal = 'ERROR';

    return convertedVal

def parseSerialData(hSerial):
    hSerial.write("g\n")

    display_flag = True;
    while (display_flag):

        serialInput=hSerial.readline()
        tag=serialInput[0:4]
        if(tag=="SNSR"):
            timeStamp=int(serialInput[5:15])
            sensorList=serialInput.split()
            sensorList=sensorList[1:len(sensorList)]
            sensorList.insert(0,timeStamp)

            #Write to datalog file
            T=time.time()
            ts1 = datetime.datetime.fromtimestamp(T).strftime('%Y_%m_%d_%H')
            dataFile=open(datadir+"/datalog_" + ts1 + ".tsv", 'a+')
            ts2 = datetime.datetime.fromtimestamp(T).strftime('%Y%m%d%H%M%S')
            dataFile.write("%15f,%15d,%15d,%15d,%15d,%15d,%15d,%15d,%15d\n" % (int(ts2),int(sensorList[0]),float(sensorList[1]),float(sensorList[2]),float(sensorList[3]),float(sensorList[4]),float(sensorList[5]),float(sensorList[6]),float(sensorList[7]))) 

            display_flag=False
            return sensorList  
            
        elif (tag == "DBUG") or (tag == "ATTN") or (tag == "EROR"):
            print("\n\033[1;46m"+serialInput+"\033[1;m")
            recordNote(serialInput)
        
def recordNote(txt):
    #print("Recording note: "+ str(txt))
    T=time.time()
    ts1 = datetime.datetime.fromtimestamp(T).strftime('%Y_%m_%d_%H')
    noteFile=open(notedir+"/notelog_" + ts1 + ".tsv", 'a+')
    ts2 = datetime.datetime.fromtimestamp(T).strftime('%Y%m%d%H%M%S')
    #noteFile.write(ts2+" "+txt+"\n")
    
def send2board(hSerial, txt):
    print("Sending: " + str(txt) + "\n")
    hSerial.write(txt+"\n")
    
