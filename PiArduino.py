import binascii
import spidev
import time

spi = spidev.SpiDev()

#set which arduino to talk to. slave(0) for arduino 1 and slave(1) for arduino 2
def setSlave(PiBus):
  device = 0
  bus = PiBus
  spi.open(device,bus)
  spi.mode = 0
  spi.max_speed_hz = 115200

def transmit(message):  
  try:
    while True:
      print (message)
      tx = spi.writebytes([message])
      time.sleep(0.3)
  finally:
    tx = spi.writebytes([ord('S')])
    spi.close()


def fwd():
  setSlave(1)
  cmd = ord('F')
  #print b
  print cmd
  transmit(cmd)
  #cmd = ord(param) -- do some math on the param to separate different speeds. 
  #Maybe >100 one speed <100 another set speed
  
def back():
  setSlave(1)
  cmd = ord('B')
  #print b
  print cmd
  transmit(cmd)          

def left():
  setSlave(1)
  cmd = ord('L')
  #print b
  print cmd
  transmit(cmd) 
          
def right():
  setSlave(1)
  cmd = ord('R')
  #print b
  print cmd
  transmit(cmd)     

def stop():
  setSlave(1)
  cmd = ord('S')
  #print b
  print cmd
  transmit(cmd)       

def LineFollow():
  setSlave(1)
  cmd = ord('T') #for tape follow
  #print b
  print cmd
  transmit(cmd)

def ObjectDetection():
  setSlave(0)
  cmd = ord('O') 
  #print b
  print cmd
  transmit(cmd)            
                  
