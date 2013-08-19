#!/usr/bin/python
# coding=utf-8

############################################################
# author: Juergen Maier
# 25.3.2013
# project: RFID reader implementation for robot
############################################################
# this is the main module initializing the other at startup,
# also the hardware interface to the reader is set up here,
# after that it checks for new messages and starts a read
# process if new data is available
############################################################

import serial
import sys
import time
from rfid_reader_interface import *
from rfid_cmd_receiver import *
from rfid_tag_sender import *

BAUDRATE=19200
PARITY=serial.PARITY_NONE
STOPBIT=serial.STOPBITS_ONE

# how long the program shall be sleeping between 2
# checks for new input data in seconds
SLEEP_TIME=0.1

############################################################
# prints usage of program
############################################################

def usage():
    print("###### USAGE ###############")
    print("rfid_reader.py <serialPort>")
    print("e.g. python rfid_reader.py /dev/tty1")

############################################################
# opens serial interface on specified port
############################################################

def open_interface(port):
    ser=serial.Serial(port,baudrate=BAUDRATE, parity =
                      PARITY, stopbits=STOPBIT)

    # it may be necessary to comment the following line in,
    # however on my MAC this threw an error
    #ser.open()

    return ser

############################################################
# closes interface
############################################################

def close_interface(ser):
    ser.close()


############################################################
# read function, is called each time a function inside
# rfid_reader_interface.py wants to read from the reader
############################################################

def read_from_ser() :
    input= ser.read()
    return bytearray(input)

############################################################
# main program
############################################################

if __name__ == '__main__':

    global ser

    if len(sys.argv) != 2 :
        usage()
        exit(1)

    # start ROS topic
    openPublisher("rfid_tag_id","rfid_reader")

    # start ROS service
    rfid_settings_start_server()

    # open serial interface on defined port
    ser=open_interface(sys.argv[1])

    # set reference to read and write fct
    set_write_fct(ser.write)
    set_read_fct(read_from_ser)

    while 1:
	# if shutdown command was received quit
        if got_shutdown() == True :
            break

	# as long as input data are waiting at the reader
	# interface continue reading
        while ser.inWaiting() != 0 :
            readMessage()

	# sleep prevents busy waiting
        time.sleep(SLEEP_TIME)

    close_interface(ser)
