#!/usr/bin/python
import sys
import serial
from socket import *
#from FlightGear import FlightGear
import time


def rotary_knobs(element):
    
    vs = element[0].split(':')[1] + '.0'
    #print "vs:" , vs
    #fg['/flight-management/fcu-values/vs'] = vs
    #fg['/autopilot/settings/vertical-speed-fpm'] = vs
    baro = element[1].split(':')[1] + '.0'
    spd = element[2].split(':')[1]  
    #fg['/flight-management/fcu-values/ias'] = spd
    #fg['/autopilot/settings/target-speed-kt'] = spd
    hdg = element[3].split(':')[1] + '.0' 
    #fg['/flight-management/fcu-values/hdg'] = hdg
    #fg['/autopilot/settings/heading-bug-deg'] = hdg
    alt = element[4].split(':')[1] 
    #fg['/flight-management/fcu-values/alt'] = alt
    #fg['/autopilot/settings/target-altitude-ft'] = alt 
    
    outline = spd + ',' + hdg + ',' + alt +',' + vs +','
    
    return outline
    #UDPSock.sendto(outline,addr)


def efis_pushbuttons(byte,outline):
  #bytes[9]
    if (ord(byte) & (1<<2)) == 0:
     # fg['/instrumentation/efis[1]/inputs/ARPT'] = 1
      outline+='1,'
    else:
     # fg['/instrumentation/efis[1]/inputs/ARPT'] = 0
      outline+='0,'
    if (ord(byte) & (1<<3)) == 0:
     # fg['/instrumentation/efis[1]/inputs/NDB'] = 1
      outline+='1,'
    else:
      outline+='0,'
     # fg['/instrumentation/efis[1]/inputs/NDB'] = 0
    if (ord(byte) & (1<<4)) == 0:
      outline+='1,'
     # fg['/instrumentation/efis[1]/inputs/VORD'] = 1
    else:
      outline+='0,'
     # fg['/instrumentation/efis[1]/inputs/VORD'] = 0
    if (ord(byte) & (1<<5)) == 0:
      outline+='1,'
     # fg['/instrumentation/efis[1]/inputs/WPT'] = 1
    else:
      outline+='0,'
     # fg['/instrumentation/efis[1]/inputs/WPT'] = 0
    if (ord(byte) & (1<<6)) == 0:
      outline+='1'
     # fg['/instrumentation/efis[1]/inputs/CSTR'] = 1
    else:
      outline+='0'
     # fg['/instrumentation/efis[1]/inputs/CSTR'] = 0

    return outline 
     

    
host = "i3"
port = 21567
#buf = 1024
addr = (host,port)
UDPSock = socket(AF_INET,SOCK_DGRAM)

bitfield =[]



tty="/dev/ttyUSB0"
print "tty is ", tty


try:
	ser = serial.Serial(tty,19200)
except:
	print "Error connecting to " , tty
	

prevline=""
while 1:
  outline=''
  line=ser.readline()
  #laenge pruefen - ist variabel
  #print len(line)
  if line != prevline:  #some value changed
    print line
    element = line.split(',')
    
    # 4knobs
    #outline = rotary_knobs(element)
    # todo: com,nav...
    
    # 10 bytes
    #bytes = element[9].split(':')[1]
    # 5 pushbuttons
    #outline = efis_pushbuttons(bytes[9],outline)
   
    # properties ?
    #rafi_pushbuttons(bytes[8])
    
    #nd_knobs(bytes[2],bytes[3])
    #nd_switches(bytes[1])
    
    #outline ="230.0,179.0\n"
    
    #outline = outline + '\n'
    #UDPSock.sendto(outline,addr)

    #print '*'
    outline=line
    UDPSock.sendto(outline,addr)
    prevline=line

exit 
