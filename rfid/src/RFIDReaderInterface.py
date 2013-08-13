#!/usr/bin/python
# coding=utf-8

############################################################
# author: Juergen Maier
# 25.3.2013
# project: RFID reader implementation for robot
############################################################
# this modules handles the complete communication with the
# RFID reader: Reader Cube from RFID Innovations
#
# before using the functions in this module the functions
# set_write_fct() and set_read_fct() have to be called
############################################################

### import ##############################################

from rfid_tag_sender import *
import time
import struct

### constants ##############################################

doDebug=True

RFE_FRAME_START_BYTE1=0x52
RFE_FRAME_START_BYTE2=0x46
RFE_FRAME_START_BYTE3=0x45

RFE_COMMAND_START_BYTE=0x01
RFE_LENGTH_START_BYTE=0x02
RFE_PAYLOAD_START_BYTE=0x03
RFE_CHECKSUM_START_BYTE=0x04

RFE_RET_SUCCESS=0x00
RFE_RET_RESULT_PENDING = 0x01
RFE_RET_ERR_OP_NOT_SUPPORTED=0x50
RFE_RET_ERR_UNKNOWN_ERR=0x51
RFE_RET_ERR_ON_EXEC_OP=0x52
RFE_RET_ERR_COULD_NOT_WRITE=0x53
RFE_RET_ERR_WRONG_PARAM_COUNT=0x54
RFE_RET_ERR_WRONG_PARAM=0x55
RFE_RET_TMI_TAG_UNREACHABLE = 0xA0
RFE_RET_TMI_MEM_OVERRUN = 0xA1
RFE_RET_TMI_MEM_LOCKED = 0xA2
RFE_RET_TMI_INSUFFICIENT_POWER = 0xA3
RFE_RET_TMI_WRONG_PASSWORD = 0xA4

RFE_STATE_IDLE=0x00
RFE_STATE_REBOOTING=0x01
RFE_STATE_SCANNING=0x10
RFE_STATE_WRITING=0x11
RFE_STATE_READING=0x12

CYCLIC_ID_LENGTH_POS=1
CYCLIC_ID_START_POS=2

SINGLE_INV_STATUS_POS=0
SINGLE_INV_COUNT_POS=1
SINGLE_INV_HEADER_CNT=3

SINGLE_INV_TAG_LENGTH_POS=0
SINGLE_INV_ID_LENGTH_POS=2
SINGLE_INV_ID_START=3

START_BYTE_RSSI=0x02
RSSI_NO_VALUE=0

BOOTUP_STARTING=0x00
BOOTUP_FINISHED=0x01

POWER_SAVE_DATA_BEG="000103"
POWER_SAVE_ON="01"
POWER_SAVE_OFF="00"
POWER_SAVE_SLEEP_DEF="00FF"

#determines the number of sleep cycles when waiting for
# response
RESP_TIMEOUT_COUNT=20
# determines the length of one sleep cycle when waiting
# for response
RESP_TIMEOUT_TIME=0.1

FREQ_MODE_STATIC_UP=0x00
FREQ_MODE_RANDOM=0x01

CHECKSUM_INIT=(RFE_FRAME_START_BYTE1^RFE_FRAME_START_BYTE2^
    RFE_FRAME_START_BYTE3^RFE_COMMAND_START_BYTE^
    RFE_LENGTH_START_BYTE^RFE_CHECKSUM_START_BYTE)

MSG_LENGTH_INIT = 3 + 1 + 2 + 1 + 1 + 1 + 1
# preamble(3) + start command(1) + command(2) + start
# length(1) + length(1) + start checksum(1) + checksum(1)

### text field definitions ######################################

retText={}
retText[RFE_RET_SUCCESS]="no error occured"
retText[RFE_RET_RESULT_PENDING]="result pending"
retText[RFE_RET_ERR_OP_NOT_SUPPORTED]="operation not supported"
retText[RFE_RET_ERR_UNKNOWN_ERR]="Unknown error"
retText[RFE_RET_ERR_ON_EXEC_OP]="Operation could not be executed"
retText[RFE_RET_ERR_COULD_NOT_WRITE]="Value could not be written"
retText[RFE_RET_ERR_WRONG_PARAM_COUNT]="Wrong parameter count"
retText[RFE_RET_ERR_WRONG_PARAM]="Wrong parameter"
retText[RFE_RET_TMI_TAG_UNREACHABLE]="TMI Tag unreachable"
retText[RFE_RET_TMI_MEM_OVERRUN]="TMI memory overrung"
retText[RFE_RET_TMI_MEM_LOCKED]="TMI memory locked"
retText[RFE_RET_TMI_INSUFFICIENT_POWER]="TMI insufficient power"
retText[RFE_RET_TMI_WRONG_PASSWORD]="TMI wrong password"

stateText={}
stateText[RFE_STATE_IDLE]="idle"
stateText[RFE_STATE_SCANNING]="scanning"
stateText[RFE_STATE_REBOOTING]="rebooting"
stateText[RFE_STATE_WRITING]="writing"
stateText[RFE_STATE_READING]="reading"

bootUpText={}
bootUpText[BOOTUP_STARTING]="starting"
bootUpText[BOOTUP_FINISHED]="finished"

freqModeText={}
freqModeText[FREQ_MODE_STATIC_UP]="static up"
freqModeText[FREQ_MODE_RANDOM]="random"

### callback dictionary definition ############################

# if a field in this dictionary is set to True a command
# with an adequate number has been received
gotResponse={}
gotResponse['cyclicInv']=False
gotResponse['singleInv']=False
gotResponse['writeToTag']=False
gotResponse['serialNr']=False
gotResponse['hardwareRev']=False
gotResponse['softwareRev']=False
gotResponse['paramSet']=False
gotResponse['statusReg']=False
gotResponse['readerState']=False
gotResponse['att']=False
gotResponse['freq']=False
gotResponse['sens']=False
gotResponse['setFreq']=False
gotResponse['setAntennaPow']=False
gotResponse['setAtt']=False
gotResponse['setSens']=False
gotResponse['saveSet']=False
gotResponse['readFromTag']=False

# the text that is delivered back to the caller is stored in this
# dictionary
responseText={}
responseText['cyclicInv']=""
responseText['singleInv']=""
responseText['writeToTag']=""
responseText['serialNr']=""
responseText['hardwareRev']=""
responseText['softwareRev']=""
responseText['paramSet']=""
responseText['statusReg']=""
responseText['readerState']=""
responseText['att']=""
responseText['freq']=""
responseText['sens']=""
responseText['setFreq']=""
responseText['setAntennaPow']=""
responseText['setAtt']=""
responseText['setSens']=""
responseText['saveSet']=""
responseText['readFromTag']=""

############################################################
# initialises the message structure with the fields
# command and data
############################################################

def init_msg_structure() :
	msg_struct = {}
	msg_struct['command'] = bytearray.fromhex('')
	msg_struct['data'] = bytearray.fromhex('')
	
	return msg_struct

############################################################
# prints passed text to output if debug mode is activated
############################################################

def print_debug(msg) :

    if doDebug == True :
        print "DEBUG: " + msg

############################################################
# converts a Bytearray to a String holding the values in
# hex format
############################################################

def msg_to_string (msg) :

    msg_str=''

    if len(msg) == 1 :
        msg_str = '%02X' % msg[0]
        return msg_str

    for num in msg :
        msg_str += '%02X' % num

    return msg_str

############################################################
# converts Bytearray to String holding values in hex format
# and passes result to fct. print_debug
############################################################

def print_msg(msg) :

    msg_str=''

    if len(msg) == 1 :
        msg_str = '%02X' % msg[0]
        return msg_str

    for num in msg :
        msg_str += '%02X' % num

    print_debug(msg_str)

############################################################
# converts a four digits hex string into a signed 16 bit
# integer by building the 2s complement
############################################################

def hex_to_int16(text) :

    return (-int("FFFF",16)+int(text,16)-1)

############################################################
# returns the text for the corresponding return code or
# unknown if return code is not known
############################################################

def get_ret_text (code) :

    if code in retText :
        return retText[code]
    else :
        return "unknown"

############################################################
# returns the text for the corresponding state code or
# unknown if state code is not known
############################################################

def get_state_text (code) :

    if code in stateText :
        return stateText[code]
    else :
        return "unknown"

############################################################
# separates IDs out of the data block received from a cyclic
# inventory interrupt
############################################################

def get_id_from_cyclic_interrupt(msg) :

    id=""

	# get length of actual ID
    length=msg[CYCLIC_ID_LENGTH_POS]

	# read ID
    for i in range(length) :
        id += "%02X" % msg[i+CYCLIC_ID_START_POS]

	# if more data than just ID present
    if len(msg) > (length+2) :
		# check if additional data RSSI values
        if msg[length+CYCLIC_ID_START_POS] == START_BYTE_RSSI :
            id += " "
            i = length+CYCLIC_ID_START_POS+1
			# read RSSI values
            while i < len(msg) :
	            id +="%02X" % msg[i]
	            i += 1
        else :
	         id +=" %02X" % RSSI_NO_VALUE
    else :
        id += " %02X" % RSSI_NO_VALUE

    return id

############################################################
# separates IDs out of the data block received from a single
# inventory interrupt
############################################################

def get_id_from_single_inventory(msg) :

    id=""
    print_debug("ret code: " +
		get_ret_text(msg[SINGLE_INV_STATUS_POS]) )

	# how many IDs were delivered
    IDcount=msg[SINGLE_INV_COUNT_POS]

	# strip header
    msg=msg[SINGLE_INV_HEADER_CNT:] 

    for i in range(IDcount) :
        lengthTag=msg[SINGLE_INV_TAG_LENGTH_POS]
        lengthID=msg[SINGLE_INV_ID_LENGTH_POS]

		# read ID
        for j in range(lengthID):
            id += '%02X' % msg[j + SINGLE_INV_ID_START]

		# if RSSI value present
        if lengthTag > (lengthID+2) : 
            if msg[SINGLE_INV_ID_START+lengthID] == START_BYTE_RSSI :
	            id += " %02X" %	msg[SINGLE_INV_ID_START+lengthID+1]
            else:
                id +=" %02X" % RSSI_NO_VALUE
        else:
            id +=" %02X" % RSSI_NO_VALUE

        if i != IDcount-1 : # not the last ID
            msg=msg[lengthTag+1:] # cut off processed Tag Info
            id+=" "

    return id


### process functions ######################################
# these functions handle the incoming messages after they
# had been received and identified, each code number has
# its own process function
############################################################

############################################################
# is called if no process function had been defined for
# the received code
############################################################

def process_fct_default(msg) :
    print_debug("process_fct_default")
    print_msg(msg['data'])

############################################################
# is sent from reader each time the state changes
############################################################

def process_fct_9003(msg) :
    print_debug("process_fct_9003")
    print_msg(msg['data'])

    if msg['data'][0] in stateText :
        print_debug("state of reader changed, new status: " +
	       get_state_text(msg['data'][0]) )
    else :
        print_debug("state of reader changed to unknown state")

############################################################
# is sent in cyclic inventory mode, contains exactly one
# Tag ID
############################################################

def process_fct_9002(msg) :
    print_debug("got RFID Tag ID")
    
    print_msg(msg['data'])
    id=get_id_from_cyclic_interrupt(msg['data'])
    print_debug("ids " + id)
    publishMessage(id)

############################################################
# is sent every time the reader detects an error
############################################################

def process_fct_9004(msg) :
    print_debug("reader detected an error")
    print_debug("status register of reader:")
    print_msg(msg['data'])

############################################################
# is sent every time the reader boots up
############################################################

def process_fct_9005(msg) :
    print_debug("reader sent boot up interrupt")

    if msg['data'][0] in bootUpText :
        print_debug("bootup status code: " +
	       bootUpText[msg['data'][0]])
    else :
        print_debug("bootup status code not known")

############################################################
# single inventory, returns all IDs currently in range
############################################################

def process_fct_5001(msg) :
    print_debug("single inventory returned:")
    
    print_msg(msg['data'])
    id=get_id_from_single_inventory(msg['data'])
    print_debug("ids: " + id)
    publishMessage(id)

    ids=id.split(' ')

    responseText['singleInv']="single inventory " + \
    "finished, IDs read:\n"

	# print each ID in own line
    for i in range(len(ids)/2) :
        responseText['singleInv']+="ID #" + str(i+1) \
        + ": " + ids[i*2] + "\n"

    gotResponse['singleInv']=True

############################################################
# is returned after cyclic inventory is activated or
# deactivated, contains error code
############################################################

def process_fct_5002(msg) :
    print_debug("return code of last cyclic inventory: "+ \
    get_ret_text(msg['data'][0]) )

    responseText['cyclicInv']="cyclic inventory command " + \
    "finished: " + get_ret_text(msg['data'][0])
    gotResponse['cyclicInv']=True

############################################################
# is returned after read from tag command was issued
############################################################

def process_fct_5003(msg) :
    print_debug("return code of read from tag: "+ \
    get_ret_text(msg['data'][0]) )

    if msg['data'][0] != RFE_RET_SUCCESS :
        responseText['readFromTag']="read from tag command" \
        " returned: " + get_ret_text(msg['data'][0])
        gotResponse['readFromTag']=True
        return

    responseText['readFromTag']="read from tag returned data: " + \
    msg_to_string(msg['data'][2:])
    gotResponse['readFromTag']=True

############################################################
# is returned after write to tag command was issued
############################################################

def process_fct_5004(msg) :
    print_debug("return code of write to tag: "+ \
    get_ret_text(msg['data'][0]) )

    responseText['writeToTag']="write to tag command " + \
    "finished: " + get_ret_text(msg['data'][0])
    gotResponse['writeToTag']=True

############################################################
# returns the serial number of the reader
############################################################

def process_fct_0101(msg) :
    print_debug("got serial number of device: ")
    
    print_msg(msg['data'])

    responseText['serialNr']="serial number of device: " + \
    msg_to_string(msg['data'])
    gotResponse['serialNr']=True

############################################################
# returns hardware revision
############################################################

def process_fct_0103(msg) :
    print_debug("got hardware rev: ")
    print_msg(msg['data'])

    text=msg_to_string(msg['data'])

    responseText['hardwareRev']="hardware revision is " + \
    text[4:6] + "." + text[6:]
    gotResponse['hardwareRev']=True

############################################################
# returns software revision
############################################################

def process_fct_0104(msg) :
    print_debug("got software rev: ")
    print_msg(msg['data'])

    text=msg_to_string(msg['data'])

    responseText['softwareRev']="software revision of App is " + \
    text[0:2] + "." + \
    text[2:4] + " and of the Kernel " + \
    text[4:6] + "." + \
    text[6:]
    gotResponse['softwareRev']=True

############################################################
# is returned after the set parameter command was issued
############################################################

def process_fct_0330(msg) :
    print_debug("return code of last set parameter: " + \
    get_ret_text(msg['data'][0]) )

    responseText['paramSet']="command " + \
    "finished: " + get_ret_text(msg['data'][0])
    gotResponse['paramSet']=True

############################################################
# returns status register
############################################################

def process_fct_0108(msg) :
    print_debug("got status register: ")
    print_msg(msg['data'])

    responseText['statusReg']="status register is " + \
    msg_to_string(msg['data'])
    gotResponse['statusReg']=True

############################################################
# returns reader state
############################################################

def process_fct_0107(msg) :
    print_debug("state of reader received")
    print_msg(msg['data'])
    if msg['data'][0] in stateText :
        print_debug("state of reader: " + \
	       stateText[msg['data'][0]])
        responseText['readerState']="reader is currently in state "+ \
        stateText[msg['data'][0]]
    else :
        print_debug("state of reader is unknown state")
        responseText['readerState']="reader is currently in" + \
        "unknown state"
   
    gotResponse['readerState']=True

############################################################
# returns attenuation
############################################################

def process_fct_0201(msg) :
    print_debug("got attenuation: ")
    print_msg(msg['data'])

	# if command was not successful only return error code
    if msg['data'][0] != RFE_RET_SUCCESS :
        responseText['att']="get attenuation command returned" + \
        get_ret_text(msg['data'][0])
    else :
        responseText['att']="max attenuation is 0x" + \
        msg_to_string(msg['data'][1:3]) + \
        ", current attenuation is 0x" + msg_to_string(msg['data'][3:])

    gotResponse['att']=True

############################################################
# returns frequency
############################################################

def process_fct_0202(msg) :
    print_debug("got frequency: ")
    print_msg(msg['data'])

	# if command was not successful only return error code
    if msg['data'][0] != 0 :
        responseText['freq']="get frequency command returned" + \
        get_ret_text(msg['data'][0])
        gotResponse['freq']=True
        return

	# determine frequency mode
    responseText['freq']= "current mode: "
    if msg['data'][1] in freqModeText :
        responseText['freq']+= freqModeText[msg['data'][1]] + "\n"
    else :
        responseText['freq']+= "unknown\n"

    responseText['freq']+= "max frequencies: " \
    + str(msg['data'][2]) + "\n"

	# print each frequency value in own row
    for i in range(msg['data'][3]) :
        pos=4+i*3
        freq=0
		#convert hex value to int
        for j in range(3) :
            freq=(freq<<8) + msg['data'][pos+j]

        responseText['freq']+="frequency " + str(i+1) + ": 0x" \
        + msg_to_string(msg['data'][pos:pos+3]) + "(" + \
        str(freq) + " kHz)\n"

    gotResponse['freq']=True

############################################################
# returns sensitivity
############################################################

def process_fct_0203(msg) :
    print_debug("got sensitivity: ")
    print_msg(msg['data'])

	# if command was not successful only return error code
    if msg['data'][0] != RFE_RET_SUCCESS :
        responseText['sens']="get sensitivity command returned" + \
        get_ret_text(msg['data'][0])
        gotResponse['sens']=True
        return

	# convert hex to int values
    #values=struct.unpack("hhh",msg['data'][1:])
    text=msg_to_string(msg['data'])
    value1=hex_to_int16(text[2:6],16)
    value2=hex_to_int16(text[6:10],16)
    value3=hex_to_int16(text[10:14],16)

    responseText['sens'] = "max sensitivity: 0x" + text[2:6] \
    + " (" + str(value1) + " dBm)\n"
    responseText['sens'] += "min sensitivity: 0x" + text[6:10] \
    + " (" + str(value2) + " dBm)\n"
    responseText['sens'] += "current sensitivity: 0x" + \
    text[10:14] + " (" + str(value3) + " dBm)"        

    gotResponse['sens']=True

############################################################
# is returned after set frequency
############################################################

def process_fct_0282(msg) :
    print_debug("return code of last set frequency: " + \
    get_ret_text(msg['data'][0]))

    responseText['setFreq']="set frequency command finished: " + \
    get_ret_text(msg['data'][0])
    gotResponse['setFreq']=True

############################################################
# return code of set antenna power
############################################################

def process_fct_0303(msg) :
    print_debug("return code of last set antenna power: " + \
    get_ret_text(msg['data'][0]))

    responseText['setAntennaPow']="set antenna power command " + \
    "finished: " + get_ret_text(msg['data'][0])
    gotResponse['setAntennaPow']=True

############################################################
# return code of set attenuation
############################################################

def process_fct_0281(msg) :

    print_debug("return code of last set attenuation: " + \
    get_ret_text(msg['data'][0]))

    responseText['setAtt']="set attenuation command " + \
    "finished: " + get_ret_text(msg['data'][0])
    gotResponse['setAtt']=True

############################################################
# return code of set sensitivity
############################################################

def process_fct_0283(msg) :
    print_debug("set sensitivity return: ")
    print_msg(msg['data'])

    responseText['setSens']="set sensitivity command" + \
    "finished: " + get_ret_text(msg['data'][0]) + "\n"

    text=msg_to_string(msg['data'][1:])
	# convert hex value to int
    values=struct.unpack("bb",msg['data'][1:])

    responseText['setSens']+="sensitivity set to value " + \
    text + " (" + str(values[1]) +  " dBm)"

    gotResponse['setSens']=True

############################################################
# return code of save settings
############################################################

def process_fct_0321(msg) :
    print_debug("return code of last save settings: " + \
    get_ret_text(msg['data'][0]))

    responseText['saveSet']="save settings command " + \
    "finished: " + get_ret_text(msg['data'][0])
    gotResponse['saveSet']=True

############################################################
# here the process functions are matched with the return
# code that should trigger them
############################################################

process_fct={}
process_fct['0101']=process_fct_0101
process_fct['0103']=process_fct_0103
process_fct['0104']=process_fct_0104
process_fct['5001']=process_fct_5001
process_fct['5002']=process_fct_5002
process_fct['5003']=process_fct_5003
process_fct['5004']=process_fct_5004
process_fct['9002']=process_fct_9002
process_fct['9003']=process_fct_9003
process_fct['9004']=process_fct_9004
process_fct['9005']=process_fct_9005
process_fct['0330']=process_fct_0330
process_fct['0108']=process_fct_0108
process_fct['0107']=process_fct_0107
process_fct['0201']=process_fct_0201
process_fct['0202']=process_fct_0202
process_fct['0203']=process_fct_0203
process_fct['0282']=process_fct_0282
process_fct['0283']=process_fct_0283
process_fct['0281']=process_fct_0281
process_fct['0303']=process_fct_0303
process_fct['0321']=process_fct_0321
process_fct['default']=process_fct_default

############################################################
##### messages structures ##################################
# the structure used to generate the messages that are sent
# to the reader get defined here, generated are these
# messages in fct get_msg()
############################################################

msg_structure = bytearray.fromhex('52 46 45 01 02 03 04')

#get serial number
serial_number = init_msg_structure()
serial_number['command']=bytearray.fromhex('01 01')

#get hardware revision
hardware_rev = init_msg_structure()
hardware_rev['command']=bytearray.fromhex('01 03')

#get software revision
software_rev = init_msg_structure()
software_rev['command']=bytearray.fromhex('01 04')

#get current state
get_curr_state= init_msg_structure()
get_curr_state['command']=bytearray.fromhex('01 07')

#get status register
get_status_reg= init_msg_structure()
get_status_reg['command']=bytearray.fromhex('01 08')

#get attenuation
get_att= init_msg_structure()
get_att['command']=bytearray.fromhex('02 01')

#get frequency
get_freq= init_msg_structure()
get_freq['command']=bytearray.fromhex('02 02')

#get sensitivity
get_sens= init_msg_structure()
get_sens['command']=bytearray.fromhex('02 03')

#set frequency
set_freq=init_msg_structure()
set_freq['command']=bytearray.fromhex('02 82')

#reboot
reboot= init_msg_structure()
reboot['command']=bytearray.fromhex('03 01')

#antenna power off
set_antenna_pow_off= init_msg_structure()
set_antenna_pow_off['command']=bytearray.fromhex('03 03')
set_antenna_pow_off['data']=bytearray.fromhex('00')

#antenna power on
set_antenna_pow_on= init_msg_structure()
set_antenna_pow_on['command']=bytearray.fromhex('03 03')
set_antenna_pow_on['data']=bytearray.fromhex('01')

# set attenuation
set_att= init_msg_structure()
#set_att['command']=bytearray.fromhex('03 04')
set_att['command']=bytearray.fromhex('02 81')

# set sens
set_sens= init_msg_structure()
set_sens['command']=bytearray.fromhex('02 83')

# save settings
save_settings= init_msg_structure()
save_settings['command']=bytearray.fromhex('03 21')

# single inventory
single_inv = init_msg_structure()
single_inv['command']=bytearray.fromhex('50 01')

# switch cyclic inventory on for specific time
cyclic_inv_on_time= init_msg_structure()
cyclic_inv_on_time['command']=bytearray.fromhex('50 02')

# switch cyclic inventory on
cyclic_inv_on= init_msg_structure()
cyclic_inv_on['command']=bytearray.fromhex('50 02')
cyclic_inv_on['data']=bytearray.fromhex('01')

# switch cyclic inventory off
cyclic_inv_off= init_msg_structure()
cyclic_inv_off['command']=bytearray.fromhex('50 02')
cyclic_inv_off['data']=bytearray.fromhex('00')

# write to tag
write_to_tag=init_msg_structure()
write_to_tag['command']=bytearray.fromhex('50 04')

# read from tag
read_from_tag=init_msg_structure()
read_from_tag['command']=bytearray.fromhex('50 03')

# switch RSSI on
set_rssi_on = init_msg_structure()
set_rssi_on['command']=bytearray.fromhex('03 30')
set_rssi_on['data'] = bytearray.fromhex('00 02 01 01')

# switch RSSI off
set_rssi_off = init_msg_structure()
set_rssi_off['command']=bytearray.fromhex('03 30')
set_rssi_off['data'] = bytearray.fromhex('00 02 01 00')

# set power save on/off
set_power_save = init_msg_structure()
set_power_save['command']=bytearray.fromhex('03 30')

# defines how the reader reacts if a tag is detected,
# with this settings it sends the tag immediatly
set_tag_behav_imm = init_msg_structure()
set_tag_behav_imm['command']=bytearray.fromhex('03 30')
set_tag_behav_imm['data'] = bytearray.fromhex('00 03 01 00')

# defines how the reader reacts if a tag is detected,
# with this settings it sends the tag only once per cyclic inv
set_tag_behav_once = init_msg_structure()
set_tag_behav_once['command']=bytearray.fromhex('03 30')
set_tag_behav_once['data'] = bytearray.fromhex('00 03 01 01')

### functions ##############################################

############################################################
# creates a message with the desired length and sets the
# first bytes, returns the msg itself as well as the
# position where the next value has to be written
############################################################

def init_msg(msg_length) :
    msg=bytearray(msg_length)
    i=0

    msg[i] = RFE_FRAME_START_BYTE1
    i+=1

    msg[i] = RFE_FRAME_START_BYTE2
    i+=1

    msg[i] = RFE_FRAME_START_BYTE3
    i+=1

    return msg, i

############################################################
# generates from a message structure defined above a
# message that can then be sent to the reader
# returns the complete message at the end
############################################################

def get_msg(cmd_struct) :

    # init checksum
    msg_checksum = CHECKSUM_INIT

    msg_length = MSG_LENGTH_INIT

	# if data to send available alter length
    if len(cmd_struct['data']) != 0 :
        msg_length += len(cmd_struct['data']) + 1

    # init msg with length byte_count
    msg,i=init_msg(msg_length)

    msg[i] = RFE_COMMAND_START_BYTE
    i+=1

    # add command bytes to message
    for command in cmd_struct['command'] :
        msg[i] = command
        msg_checksum ^= command
        i+=1

    msg[i] = RFE_LENGTH_START_BYTE
    i+=1

    msg[i] = len(cmd_struct['data'])
    msg_checksum ^= len(cmd_struct['data'])
    i+=1

    # if there is a message to send
    if len(cmd_struct['data']) != 0 :
        msg[i]=RFE_PAYLOAD_START_BYTE
        msg_checksum ^= RFE_PAYLOAD_START_BYTE
        i+=1
        for j in cmd_struct['data'] :
            msg[i]=j
            msg_checksum ^= j
            i+=1

    msg[i] = RFE_CHECKSUM_START_BYTE
    i+=1

    msg[i] = msg_checksum

    return msg

############################################################
# reads one complete message and calls the process function
# defined for the received code
############################################################

def readMessage() :
    
    input_byte= readFct()
    msg_rcv = init_msg_structure()

    i=0
	# check preamble bytes
    while i < 4 :
        if input_byte[0] != msg_structure[i] :
            str="got wrong msg structure " + input_byte[0]
            print_debug(str)
            i=0

        i+=1
        input_byte = readFct()

    checksum = CHECKSUM_INIT^input_byte[0]
    msg_rcv['command'] = '%02x' % input_byte[0]

    input_byte = readFct()
    checksum ^= input_byte[0]
    msg_rcv['command'] += '%02x' % input_byte[0]

    input_byte = readFct()

    if input_byte[0] != msg_structure[i] :
        print("received " + str(input_byte[0]) + " but expected " + \
              str(msg_structure[i]))
        return

    i+=1

    input_byte = readFct()
    checksum ^= input_byte[0]
    msg_rcv['length']=input_byte[0]

    input_byte = readFct()
    checksum ^= input_byte[0]

    if input_byte[0] != msg_structure[i] :
        print("received " + str(input_byte[0]) + " but expected " + \
              str(msg_structure[i]))
        return

    i+=1

    count=0
    msg_rcv['data'] = bytearray(msg_rcv['length'])
    
    while count < msg_rcv['length'] :
        input_byte = readFct()
        checksum ^= input_byte[0]
        msg_rcv['data'][count] = input_byte[0]
        count += 1

#    msg_rcv['data'] = getbyte(ser,msg_rcv['length'])
#    for i in msg_rcv['data']:
#        checksum ^= i

    input_byte = readFct()

    if input_byte[0] != msg_structure[i] :
        print("received " + str(input_byte[0]) + " but expected "
              + str(msg_structure[i]))
        return

    input_byte = readFct()

    if input_byte[0] != checksum :
        print("received incorrect checksum, got "
              + str(input_byte[0]) + " but expected " + str(checksum))

	# call corresponding process function or default function if
	# none defined
    if msg_rcv['command'] in process_fct :
        process_fct[msg_rcv['command']](msg_rcv)
    else :
    	print_debug("got fct code without handler fct: " + \
    		msg_rcv['command'])
    	process_fct['default'](msg_rcv)

    return

############################################################
# waits until response to sent command was received
# waits at most RESP_TIMEOUT_TIME * RESP_TIMEOUT_COUNT sec.
# afterwards a timeout is indicated
############################################################

def wait_for_response (command) :

    if command not in gotResponse :
         return "no indication possible if command suceeded"
         + "command not in got_response dictionary"

    count=0
    while count < RESP_TIMEOUT_COUNT :

         if gotResponse[command] == True :
             gotResponse[command] = False

             if command not in responseText :
                 return "command suceeded but"
                 + "no return text available"

             return responseText[command]

         count+=1
         time.sleep(RESP_TIMEOUT_TIME)

    return "timeout of command reached"

############################################################
# sets the writeFct pointer to the given value
# has to be done at the very beginning because without it
# each try to send data will result in an error
############################################################

def set_write_fct (fctPtr) :
    
    global writeFct 
    writeFct=fctPtr

############################################################
# sets the readFct pointer to the given value
# has to be done at the very beginning because without it
# each try to receive data will result in an error
############################################################

def set_read_fct(fctPtr) :

    global readFct
    readFct = fctPtr

############################################################
#### send functions ########################################
# these functions call the above described functions to
# generate the correct messages for the reader and then
# put them on the line
############################################################

############################################################
# activates sending of RSSI values
############################################################

def send_rssi_on ():
    print_debug("sending set rssi on")
    msg=get_msg(set_rssi_on)
    print_msg(msg)
    writeFct(msg)

    return "set RSSI " + wait_for_response('paramSet') 	

############################################################
# deactivates sending of RSSI values
############################################################

def send_rssi_off ():
    print_debug("sending set rssi off")
    msg=get_msg(set_rssi_off)
    print_msg(msg)
    writeFct(msg)

    return "reset RSSI " + wait_for_response('paramSet') 	

############################################################
# activates cyclic inventory mode
############################################################

def send_cyclic_on ():
    print_debug("sending set cyclic on")
    msg=get_msg(cyclic_inv_on)
    print_msg(msg)
    writeFct(msg)  

    return wait_for_response('cyclicInv') 

############################################################
# deactivates cyclic inventory mode
############################################################

def send_cyclic_off ():
    print_debug("sending set cyclic off")
    msg=get_msg(cyclic_inv_off)
    print_msg(msg)
    writeFct(msg)     

    return wait_for_response('cyclicInv')

############################################################
# activates cyclic inventory mode for a specific time
############################################################

def send_cyclic_on_time (time):
    print_debug("sending set cyclic on time")
    time="01"+time
    #data="01%08X"%(time)
    cyclic_inv_on_time['data']=bytearray.fromhex(time)
    msg=get_msg(cyclic_inv_on_time)
    print_msg(msg)
    writeFct(msg) 

    return wait_for_response('cyclicInv')

############################################################
# initiates a single inventory
############################################################

def send_single_inv ():
    print_debug("sending single inventory")
    msg=get_msg(single_inv)
    print_msg(msg)
    writeFct(msg) 

    return wait_for_response('singleInv')

############################################################
# tag IDs are sent immediately when they are sensed
############################################################

def send_tag_behav_imm ():
    print_debug("sending tag behaviour immediatly")
    msg=get_msg(set_tag_behav_imm)
    print_msg(msg)
    writeFct(msg) 
    
    return "send Tag immediatly " + wait_for_response('paramSet') 

############################################################
# tag IDs are sent only once in a cyclic inventory round
# does not work in a useful way, see documentation for details
############################################################

def send_tag_behav_once ():
    print_debug("sending tag behaviour once")
    msg=get_msg(set_tag_behav_once)
    print_msg(msg)
    writeFct(msg) 

    return "send Tag once " + wait_for_response('paramSet') 

############################################################
# initiates sending of serial number
############################################################

def send_serial_number ():
    print_debug("sending serial number request")
    msg=get_msg(serial_number)
    print_msg(msg)
    writeFct(msg) 

    return wait_for_response('serialNr')

############################################################
# initiates sending of hardware revision
############################################################

def send_hardware_rev ():
    print_debug("sending hardware revision request")
    msg=get_msg(hardware_rev)
    print_msg(msg)
    writeFct(msg) 

    return wait_for_response('hardwareRev')

############################################################
# initiates sending of software revision
############################################################

def send_software_rev ():
    print_debug("sending software revision request")
    msg=get_msg(software_rev)
    print_msg(msg)
    writeFct(msg) 

    return wait_for_response('softwareRev')

############################################################
# initiates sending of status register
############################################################

def send_status_reg ():
    print_debug("sending status register request")
    msg=get_msg(get_status_reg)
    print_msg(msg)
    writeFct(msg) 

    return wait_for_response('statusReg')

############################################################
# initiates sending of current state
############################################################

def send_curr_state ():
    print_debug("sending reader state request")
    msg=get_msg(get_curr_state)
    print_msg(msg)
    writeFct(msg) 

    return wait_for_response('readerState')

############################################################
# initiates sending of attenuation
############################################################

def send_att ():
    print_debug("sending attenuation request")
    msg=get_msg(get_att)
    print_msg(msg)
    writeFct(msg) 

    return wait_for_response('att')

############################################################
# initiates sending of frequencies
############################################################

def send_freq ():
    print_debug("sending frequency request")
    msg=get_msg(get_freq)
    print_msg(msg)
    writeFct(msg)

    return wait_for_response('freq')

############################################################
# initiates sending of sensitivity
############################################################

def send_sens ():
    print_debug("sending sensitivity request")
    msg=get_msg(get_sens)
    print_msg(msg)
    writeFct(msg)

    return wait_for_response('sens')

############################################################
# initiates reboot
############################################################

def send_reboot ():
    print_debug("sending reboot order")
    msg=get_msg(reboot)
    print_msg(msg)
    writeFct(msg)

    return "reboot command sent"

############################################################
# activates antenna
############################################################

def send_antenna_on ():
    print_debug("sending antenna on")
    msg=get_msg(set_antenna_pow_on)
    print_msg(msg)
    writeFct(msg)

    return wait_for_response('setAntennaPow')

############################################################
# deactivates antenna
############################################################

def send_antenna_off ():
    print_debug("sending antenna off")
    msg=get_msg(set_antenna_pow_off)
    print_msg(msg)
    writeFct(msg)

    return wait_for_response('setAntennaPow')

############################################################
# sets attenuation
############################################################

def send_set_att (attValue):
    print_debug("sending set attenuation")

    if len(attValue) != 4 :
        return "ERROR: sensitivity value has to have 4 " \
        + "hex digits"

    set_att['data']=bytearray.fromhex(attValue)
    msg=get_msg(set_att)
    print_msg(msg)
    writeFct(msg) 

    return wait_for_response('setAtt')

############################################################
# sets sensitivity
############################################################

def send_set_sens (sensValue):
    print_debug("sending set sensitivity")

    if len(sensValue) != 4 :
        return "ERROR: sensitivity value has to have 4 " \
        + "hex digits"

    set_sens['data']=bytearray.fromhex(sensValue)
    msg=get_msg(set_sens)
    print_msg(msg)
    writeFct(msg) 

    return wait_for_response('setSens')

############################################################
# actual settings are saved at the reader
############################################################

def send_save_settings ():
    print_debug("sending save settings")
    msg=get_msg(save_settings)
    print_msg(msg)
    writeFct(msg)

    return wait_for_response('saveSet')

############################################################
# changes ID of tag
############################################################

def send_change_id (old_new_id):
    print_debug("sending change id")

    ids=old_new_id.split(' ')
    if len(ids) != 2 :
        print_debug("got only one value for id, aborting")
        return "ERROR: specify old an new ID separated by a blank"
   
    length_id0 = "%02X" % (len(ids[0])/2)
    length_id1 = "%02X" % (len(ids[1])/2)

    data_string=length_id0+ids[0]+"01000200000000"+length_id1 \
    +ids[1]
    write_to_tag['data']=bytearray.fromhex(data_string)

    msg=get_msg(write_to_tag)
    print_msg(msg)
    writeFct(msg)

    return wait_for_response('writeToTag')

############################################################
# writes data to tag
############################################################

def send_write_to_tag (data):
    print_debug("sending write to tag")

    dataVals=data.split(' ')
    if len(dataVals) != 5 :
        print_debug("got not enough information to write" \
        + " to tag, aborting")
        return "ERROR: properties structure violated, " \
        + "must be: ID memoryBank address password data"
   
    if len(dataVals[1]) != 2:
        print_debug("memory bank has to have 2 hex digits")
        return "memory bank has to have 2 hex digits"

    if len(dataVals[2]) != 4:
        print_debug("address has to have 4 hex digits")
        return "address has to have 4 hex digits"

    if len(dataVals[3]) != 8:
        print_debug("password has to have 8 hex digits")
        return "password has to have 8 hex digits"

    length_id = "%02X" % (len(dataVals[0])/2)
    length_data = "%02X" % (len(dataVals[4])/2)

    data_string=length_id+dataVals[0]+dataVals[1]+ \
    dataVals[2]+dataVals[3]+length_data+dataVals[4]

    write_to_tag['data']=bytearray.fromhex(data_string)

    msg=get_msg(write_to_tag)
    print_msg(msg)
    writeFct(msg)

    return wait_for_response('writeToTag')

############################################################
# reads data from tag
############################################################

def send_read_from_tag (data):
    print_debug("sending read from tag")

    dataVals=data.split(' ')
    if len(dataVals) != 5 :
        print_debug("got not enough information to write" \
        + " to tag, aborting")
        return "ERROR: properties structure violated, " \
        + "must be: ID memoryBank address password dataLength"
   
    if len(dataVals[1]) != 2:
        print_debug("memory bank has to have 2 hex digits")
        return "memory bank has to have 2 hex digits"

    if len(dataVals[2]) != 4:
        print_debug("address has to have 4 hex digits")
        return "address has to have 4 hex digits"

    if len(dataVals[3]) != 8:
        print_debug("password has to have 8 hex digits")
        return "password has to have 8 hex digits"

    length_id = "%02X" % (len(dataVals[0])/2)

    data_string=length_id+dataVals[0]+dataVals[1]+ \
    dataVals[2]+dataVals[3]+dataVals[4]
    read_from_tag['data']=bytearray.fromhex(data_string)

    msg=get_msg(read_from_tag)
    print_msg(msg)
    writeFct(msg)

    return wait_for_response('readFromTag')

############################################################
# activates power save mode
############################################################

def send_power_save_on (time) :

    data=POWER_SAVE_DATA_BEG + POWER_SAVE_ON + time
    set_power_save['data']=bytearray.fromhex(data)
    msg=get_msg(set_power_save)
    print_msg(msg)
    writeFct(msg)

    return "set power save " + wait_for_response('paramSet')

############################################################
# deactivates power save mode
############################################################

def send_power_save_off () :

    data=POWER_SAVE_DATA_BEG + POWER_SAVE_OFF + POWER_SAVE_SLEEP_DEF
    set_power_save['data']=bytearray.fromhex(data)
    msg=get_msg(set_power_save)
    print_msg(msg)
    writeFct(msg)

    return "reset power save " + wait_for_response('paramSet')

############################################################
# sets new frequencies in static up mode
############################################################

def send_set_freq_static(freqs) :

    if (len(freqs) % 6) != 0  or len(freqs)==0:
        print_debug("not the right length of frequencies")
        return "ERROR: frequencies have to be 3 byte each"

    count="%02X"%(len(freqs)/6)
    data="00"+count+freqs
    set_freq['data']=bytearray.fromhex(data)
    msg=get_msg(set_freq)
    print_msg(msg)
    writeFct(msg)

    return wait_for_response('setFreq')

############################################################
# sets new frequencies in random mode
############################################################

def send_set_freq_random(freqs) :

    if (len(freqs) % 6) != 0 or len(freqs)==0:
        print_debug("not the right length of frequencies")
        return "ERROR: frequencies have to be 3 byte each"

    count="%02X"%(len(freqs)/6)
    data="01"+count+freqs
    set_freq['data']=bytearray.fromhex(data)
    msg=get_msg(set_freq)
    print_msg(msg)
    writeFct(msg)

    return wait_for_response('setFreq')

############################################################
# resets dictionary that indicates new responses
############################################################

def reset_resp_dict () :

    gotResponse['cyclicInv']=False
    gotResponse['singleInv']=False
    gotResponse['writeToTag']=False
    gotResponse['serialNr']=False
    gotResponse['hardwareRev']=False
    gotResponse['softwareRev']=False
    gotResponse['paramSet']=False
    gotResponse['statusReg']=False
    gotResponse['readerState']=False
    gotResponse['att']=False
    gotResponse['freq']=False
    gotResponse['sens']=False
    gotResponse['setFreq']=False
    gotResponse['setAntennaPow']=False
    gotResponse['setAtt']=False
    gotResponse['setSens']=False
    gotResponse['saveSet']=False
    gotResponse['readFromTag']=False

    return "Response dictionary successfully cleared."
