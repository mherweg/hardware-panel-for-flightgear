#!/usr/bin/python
import sys
import serial
from socket import *
from FlightGear import FlightGear
import time

from array import array
#from ctypes import *

host = "i3"
port = 21567
#buf = 1024
addr = (host,port)
#UDPSock = socket(AF_INET,SOCK_DGRAM)

lol = []

bitfield =[]
#byte1 = array([0,0,0,0,0,0,0,0])
mask0 = 0x01
mask1 = 0x02
mask2 = 0x04
mask3 = 0x08
mask4 = 0x10
mask5 = 0x20
mask6 = 0x40
mask7 = 0x80

tty="/dev/ttyUSB0"
print "tty is ", tty

def efis_pushbuttons(byte):
  #bytes[9]
    if (ord(byte) & (1<<2)) == 0:
      fg['/instrumentation/efis[1]/inputs/ARPT'] = 1
    else:
      fg['/instrumentation/efis[1]/inputs/ARPT'] = 0
    if (ord(byte) & (1<<3)) == 0:
      fg['/instrumentation/efis[1]/inputs/NDB'] = 1
    else:
      fg['/instrumentation/efis[1]/inputs/NDB'] = 0
    if (ord(byte) & (1<<4)) == 0:
      fg['/instrumentation/efis[1]/inputs/VORD'] = 1
    else:
      fg['/instrumentation/efis[1]/inputs/VORD'] = 0
    if (ord(byte) & (1<<5)) == 0:
      fg['/instrumentation/efis[1]/inputs/WPT'] = 1
    else:
      fg['/instrumentation/efis[1]/inputs/WPT'] = 0
    if (ord(byte) & (1<<6)) == 0:
      fg['/instrumentation/efis[1]/inputs/CSTR'] = 1
    else:
      fg['/instrumentation/efis[1]/inputs/CSTR'] = 0

def rafi_pushbuttons(byte):
  #bytes[8]
    print 'rafi', ord(byte)
    if (ord(byte) & 8) == 0:
      # A320
      fg['/autopilot/settings/engaged/'] = 1
   
    else:
      fg['/autopilot/settings/engaged/'] = 0
     
    if (ord(byte) & 16) == 0:
    
      fg['/autopilot/settings/autothrottle/'] = 1
      
    else:
      fg['/autopilot/settings/autothrottle/'] = 0
    
    if (ord(byte) & 2) == 0:
    
      fg['/autopilot/settings/flight-director/'] = 1
      
    else:
      fg['/autopilot/settings/flight-director/'] = 0
     
      
      
    
def rotary_knobs(element):
    
    vs = element[0].split(':')[1]
    fg['/flight-management/fcu-values/vs'] = vs
    fg['/autopilot/settings/vertical-speed-fpm'] = vs
    baro = element[1].split(':')[1]
    spd = element[2].split(':')[1]
    fg['/flight-management/fcu-values/ias'] = spd
    fg['/autopilot/settings/target-speed-kt'] = spd
    hdg = element[3].split(':')[1]
    fg['/flight-management/fcu-values/hdg'] = hdg
    fg['/autopilot/settings/heading-bug-deg'] = hdg
    alt = element[4].split(':')[1]
    fg['/flight-management/fcu-values/alt'] = alt
    fg['/autopilot/settings/target-altitude-ft'] = alt 
    
def nd_knobs(b2,b3):
    #print ord(b2)
    if ord(b2)  == 1:
      fg['/instrumentation/efis/nd/display-mode'] = 'ILS'
      fg['/instrumentation/efis/nd/display-mode-num'] = 0
     
    elif ord(b2)  == 2:
      fg['/instrumentation/efis/nd/display-mode'] = 'VOR'
      fg['/instrumentation/efis/nd/display-mode-num'] = 1
     
    elif ord(b2)  == 4:
      fg['/instrumentation/efis/nd/display-mode'] = 'NAV'
      fg['/instrumentation/efis/nd/display-mode-num'] = 2
     
    elif ord(b2)  == 8:
      fg['/instrumentation/efis/nd/display-mode'] = 'ARC'
      fg['/instrumentation/efis/nd/display-mode-num'] = 3
           
    elif ord(b2)  == 16:
      fg['/instrumentation/efis/nd/display-mode'] = 'PLAN'
      fg['/instrumentation/efis/nd/display-mode-num'] = 4
      
    #if ord(b3)  == 1:
    #  fg['/instrumentation/efis/nd/display-mode'] = 'ILS'
    #  fg['/instrumentation/efis/nd/display-mode-num'] = 0
          
def nd_switches(b1):
  print ord(b1)
  ob = ord(b1)
  if ob & 64  == 64:
      fg['/instrumentation/efis/input/vor-adf-1'] = 2
      
  elif ob & 8  == 8:
      fg['/instrumentation/efis/input/vor-adf-1'] = 1 
 
  if ob & 16  == 16:
      fg['/instrumentation/efis/input/vor-adf-2'] = 1
  elif ob & 32  == 32:
      fg['/instrumentation/efis/input/vor-adf-2'] = 2

  if (ob&64 == 0) and (ob&8==0):
      fg['/instrumentation/efis/input/vor-adf-1'] = 0
  if (ob&16 == 0) and (ob&32==0):
      fg['/instrumentation/efis/input/vor-adf-2'] = 0
          
    
     
          

try:
	ser = serial.Serial(tty,19200)
except:
	print "Error connecting to " , tty
	
fg = FlightGear('i3', 5500)

# Integer to Bin String
#def bin(a):
#           s=''
#           t={'0':'000','1':'001','2':'010','3':'011',
#              '4':'100','5':'101','6':'110','7':'111'}
#           for c in oct(a)[1:]:
#                    s+=t[c]
#           return s



   #lol= []
    
    #for a in range (0,10):
      #bitfield= []
      #for b in range(0,8):
      
        #bitstate = ord(bytes[a]) & (1 << b)
        #if bitstate == 0:
	  #bitfield.append(0)
	#else:
	  #bitfield.append(1)
     
      #lol.append(bitfield)

#num elements:      
 
  #sprintf( puffer, "vs:%d,",vs );
  #uart_puts( puffer );
  #sprintf( puffer, "baro:%u,",baro );
  #uart_puts( puffer );
  #sprintf( puffer, "spd:%u,hdg:%u,alt:%lu,",spd,hdg,alt );
  #uart_puts( puffer );
  #sprintf( puffer, "com:%lu,com2:%lu,nav:%lu,nav2:%lu,bytes:",com,com2,nav,nav2 );
  #uart_puts( puffer );
#//TODO: trnasponder,DME, ADF
  #for(i=0;i<8;i++){
    #uart_putc(keybyte[i] );
  #}
    #for(i=1;i<3;i++){
    #uart_putc(outword[i] );
  #}
  #uart_putc('\n' );
  
prevline=""
while 1:
  line=ser.readline()
  #laenge pruefen
  #print '.',
  if line != prevline:  #some value changed
    print line
    element = line.split(',')
    
    # 4knobs
    #rotary_knobs(element)
    # todo: com,nav...
    
    # 10 bytes
    #bytes = element[9].split(':')[1]
    # 5 pushbuttons
    #efis_pushbuttons(bytes[9])
   
    # properties ?
    #rafi_pushbuttons(bytes[8])
    
    #nd_knobs(bytes[2],bytes[3])
    #nd_switches(bytes[1])
    
    #outline ="255,177"
    #UDPSock.sendto(outline,addr)
    #print '*'
    print 'done'
    prevline=line

exit 
