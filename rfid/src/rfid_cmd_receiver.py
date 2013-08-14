#!/opt/local/bin/python
# coding=utf-8

############################################################
# author: Juergen Maier
# 25.3.2013
# project: RFID reader implementation for robot
############################################################
# this is the module offering the service in the ROS which
# accepts commands and delivers information
############################################################

#import roslib; roslib.load_manifest('rfid')
import rospy
from std_msgs.msg import String
from rfid.srv import *
from rfid_reader_interface import *

doShutdown=False

############################################################
# sets shutdown variable that tells main routine to shut
# down
############################################################

def set_shutdown () :
    global doShutdown
    doShutdown=True
    print_debug("set shutdown to true")
    return "shutdown initiated"

############################################################
# sets shutdown variable that tells main routine to shut
# down
############################################################

def do_reboot () :
    global doShutdown
    doShutdown=True
    print_debug("set shutdown to true")
    return send_reboot()

#### function dicts #########################################

# this dict defines which function is called when a specific
# command is called, the String argument of the input is not
# propagated to the function because it is not needed
commandFctNoArgs = {}
commandFctNoArgs['shutdown']=set_shutdown
commandFctNoArgs['serialNr']=send_serial_number
commandFctNoArgs['hardwareRev']=send_hardware_rev
commandFctNoArgs['softwareRev']=send_software_rev
commandFctNoArgs['rssiOn']=send_rssi_on
commandFctNoArgs['rssiOff']=send_rssi_off
commandFctNoArgs['cyclicOn']=send_cyclic_on
commandFctNoArgs['cyclicOff']=send_cyclic_off
commandFctNoArgs['singleInv']=send_single_inv
commandFctNoArgs['tagImm']=send_tag_behav_imm
commandFctNoArgs['tagOnce']=send_tag_behav_once
commandFctNoArgs['getState']=send_curr_state
commandFctNoArgs['getStatus']=send_status_reg
commandFctNoArgs['getAtt']=send_att
commandFctNoArgs['getFreq']=send_freq
commandFctNoArgs['getSens']=send_sens
commandFctNoArgs['reboot']=do_reboot
commandFctNoArgs['antennaOn']=send_antenna_on
commandFctNoArgs['antennaOff']=send_antenna_off
commandFctNoArgs['saveSettings']=send_save_settings
commandFctNoArgs['powerSaveOff']=send_power_save_off
commandFctNoArgs['resetRespDict']=reset_resp_dict

# this dict defines which function is called when a specific
# command is called, the String argument of the input is
# propagated to the function
commandFctArgs = {}
commandFctArgs['cyclicOnTime']=send_cyclic_on_time
commandFctArgs['setAtt']=send_set_att
commandFctArgs['setSens']=send_set_sens
commandFctArgs['changeID']=send_change_id
commandFctArgs['powerSaveOn']=send_power_save_on
commandFctArgs['setFreqStatic']=send_set_freq_static
commandFctArgs['setFreqRandom']=send_set_freq_random
commandFctArgs['writeToTag']=send_write_to_tag
commandFctArgs['readFromTag']=send_read_from_tag

############################################################
# this function is called whenever a command arrives, it
# then calls the corresponding function with or without
# the argument
############################################################

def handle_cmd(cmd) :

    text="got command %s and value %s" % (cmd.command,
		cmd.value)
    print_debug(text)

    if cmd.command in commandFctNoArgs :
        return commandFctNoArgs[cmd.command]()
    elif cmd.command in commandFctArgs :
        return commandFctArgs[cmd.command](cmd.value)
    else:
        print_debug("unknown settings command received")
        return "this command is not known"

############################################################
# start service that accepts commands
############################################################

def rfid_settings_start_server():
    #rospy.init_node('rfid_settings_server')
    s = rospy.Service('rfid_command', rfid_config_command,
    handle_cmd)
    print_debug("Ready to accept commands")

############################################################
# returns if the shutdown command was received
############################################################

def got_shutdown () :
    global doShutdown
    return doShutdown

############################################################

