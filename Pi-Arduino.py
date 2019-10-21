import binascii
import spidev
import time

spi = spidev.SpiDev()

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
      time.sleep(10)
      rx = spi.readbytes(2)
      print('Read: 0x(0)'.format(binascii.hexlify(bytearray(rx))))
  finally:
    spi.close()


def fwd():
  setSlave(0)
  cmd = ord('F')
  #print b
  print cmd
  transmit(cmd)
  
def back():
  setSlave(0)
  cmd = ord('B')
  #print b
  print cmd
  transmit(cmd)          

def left():
  setSlave(0)
  cmd = ord('L')
  #print b
  print cmd
  transmit(cmd) 
          
def right():
  setSlave(0)
  cmd = ord('R')
  #print b
  print cmd
  transmit(cmd)     
    
def LineFollow():
  setSlave(0)
  cmd = ord('T') #for tape follow
  #print b
  print cmd
  transmit(cmd)

def ObjectDetection():
  setSlave(1)
  cmd = ord('O') 
  #print b
  print cmd
  transmit(cmd)            
                  
