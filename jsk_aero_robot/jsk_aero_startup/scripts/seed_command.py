#!/usr/bin/env python
from struct import*
import serial
import time

class SeedCommand:
  def __init__(self,port='ttyUSB0',baud=460800):
    self.ser=serial.Serial('/dev/'+str(port), baud, timeout=0.1)

  #Binary data to Hex String data and decide number of byte
  def num2str(self,value,byte=1):
    if value < 0:
      value = 0xFFFFFF + 1 + value
      #return str(hex(value).upper()[3:][-2:].rjust(2**byte,"F"))
    return str(hex(value).upper()[2:][-2:].rjust(2**byte,"0"))

  #Binary data to Hex data and decide number of byte
  def num2hex(self,value,byte=1):
    if value < 0:
      value = 0xFFFFFF + 1 + value
    return hex(value).upper()[2:][-2:].rjust(2**byte,"0")

  #SEED UART data send
  def SEED_UART_Snd(self,SndData):

    self.ser.write(''.join(map(chr,SndData)))

    print "send\t:%s" % ''.join("{:02X}".format(ord(c)) for c in ''.join(map(chr,SndData)))
    #print ''.join("{:02X}".format(ord(c)) for c in ''.join(map(chr,SndData)))

  #####################################################################################################
  ########################## AERO Command Function  ##################################
  #####################################################################################################

  #Aero Data Read
  def AERO_Data_Read(self,disp=None):

    timeout = time.time() + 1

    while(self.ser.inWaiting() < 3):
      if(time.time() > timeout):
        print "Time Out"
        self.ser.flushInput()
        self.ser.flushOutput()
        return None
      else :
        pass

    headder1 = ord(self.ser.read(1))

    if (headder1 == 0xBF or headder1 == 0xDF or headder1 == 0xFD):
      pass
    else:
      print "Read Error. Headder is %d" % headder1
      self.ser.flushInput()
      self.ser.flushOutput()
      return None

    headder2 = ord(self.ser.read(1))
    data_length = ord(self.ser.read(1))

    timeout = time.time() + 1

    while(self.ser.inWaiting() < data_length):
      if(time.time() > timeout):
        print "Read Error. data_length is %d" % data_length
        self.ser.flushInput()
        self.ser.flushOutput()
        return None
      else:
        pass

    data_cmd = ord(self.ser.read(1))

    RcvData = self.num2str(headder1) + self.num2str(headder2) + self.num2hex(data_length) +  self.num2hex(data_cmd) \
        + ''.join("{:02X}".format(ord(c)) for c in self.ser.read(data_length))

    if disp== 1:
      print "recv\t:%s" % RcvData
    else :
      pass

    return RcvData

  #Current Set
  def AERO_Set_Current(self,ID,max_c,down_c):
    bCheckSum = 0
    bCount = 0
    bLength = 8
    SndData = bLength*[0]

    SndData[0] = 0xFD
    SndData[1] = 0xDF
    SndData[2] = 4
    SndData[3] = 0x01
    SndData[4] = ID
    SndData[5] = max_c
    SndData[6] = down_c

    #check sum
    for bCount in range(2,bLength-1,1):
      bCheckSum += int(self.num2hex(SndData[bCount]),16)

    SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

    #num to hex
    for i in range(0,bLength-1):
      SndData[i] = int(self.num2hex(SndData[i]),16)

    self.SEED_UART_Snd(SndData)

  def AERO_Snd_Position(self,ID,m_time,POS):

    bCheckSum = 0
    bCount = 0
    bLength = 10
    SndData = bLength*[0]

    SndData[0] = 0xFD
    SndData[1] = 0xDF
    SndData[2] = 6				#Data Length
    SndData[3] = 0x14			#Command
    SndData[4] = ID			#ID
    SndData[5] = POS >> 8
    SndData[6] = POS

    SndData[7] = m_time >> 8	#time[*10msec] high
    SndData[8] = m_time		#time[*10msec] low

    #check sum
    for bCount in range(2,bLength-1,1):
      bCheckSum += int(self.num2hex(SndData[bCount]),16)
    SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

    #num to hex
    for i in range(0,bLength-1):
      SndData[i] = int(self.num2hex(SndData[i]),16)

    self.SEED_UART_Snd(SndData)


  #Servo ON/OFF ->ON:1 , OFF:0
  def AERO_Snd_Servo(self,ID,data):
    bCheckSum = 0
    bCount = 0
    if(ID is 0):
      bLength = 68
      SndData = bLength*[0]

      SndData[0] = 0xFD
      SndData[1] = 0xDF
      SndData[2] = 64
      SndData[3] = 0x21

      for i in range(1,30):
        SndData[i*2 + 3 + 1] = data

    else:
      bLength = 8
      SndData = bLength*[0]

      SndData[0] = 0xFD
      SndData[1] = 0xDF
      SndData[2] = 0x04
      SndData[3] = 0x21
      SndData[4] = ID
      SndData[5] = data >> 8
      SndData[6] = data

    #check sum
    for bCount in range(2,bLength-1,1):
      bCheckSum += int(self.num2hex(SndData[bCount]),16)

    SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

    #num to hex
    for i in range(0,bLength-1):
      SndData[i] = int(self.num2hex(SndData[i]),16)

    self.SEED_UART_Snd(SndData)

  #Servo ON/OFF ->ON:1 , OFF:0
  def AERO_Get_Pos(self,ID):
    bCheckSum = 0
    bCount = 0
    bLength = 6
    SndData = bLength*[0]

    SndData[0] = 0xFD
    SndData[1] = 0xDF
    SndData[2] = 0x02
    SndData[3] = 0x41
    SndData[4] = ID

    #check sum
    for bCount in range(2,bLength-1,1):
      bCheckSum += int(self.num2hex(SndData[bCount]),16)

    SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

    #num to hex
    for i in range(0,bLength-1):
      SndData[i] = int(self.num2hex(SndData[i]),16)

    self.SEED_UART_Snd(SndData)


  #Script Run
  def AERO_Snd_Script(self,ID,data):
    bCheckSum = 0
    bCount = 0
    bLength = 8
    SndData = bLength*[0]

    SndData[0] = 0xFD
    SndData[1] = 0xDF
    SndData[2] = 0x04
    SndData[3] = 0x22
    SndData[4] = ID
    SndData[5] = data >> 8
    SndData[6] = data

    #check sum
    for bCount in range(2,bLength-1,1):
      bCheckSum += int(self.num2hex(SndData[bCount]),16)

    SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

    #num to hex
    for i in range(0,bLength-1):
      SndData[i] = int(self.num2hex(SndData[i]),16)

    self.SEED_UART_Snd(SndData)

  #Status Get
  def AERO_Get_Status(self,ID):
    bCheckSum = 0
    bCount = 0
    bLength = 6
    SndData = bLength*[0]

    SndData[0] = 0xFD
    SndData[1] = 0xDF
    SndData[2] = 0x02
    SndData[3] = 0x52
    SndData[4] = ID

    #check sum
    for bCount in range(2,bLength-1,1):
      bCheckSum += int(self.num2hex(SndData[bCount]),16)

    SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

    #num to hex
    for i in range(0,bLength-1):
      SndData[i] = int(self.num2hex(SndData[i]),16)

    self.SEED_UART_Snd(SndData)
