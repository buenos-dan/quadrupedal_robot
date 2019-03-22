#!/usr/bin/env python
# coding:utf-8
import rospy
from std_msgs.msg import String
import time

command_dict = {'s': 'SQUAT',
                't4': 'TRIAL4',
                't': 'TROT',
                'ti':'TROT_IN_PLACE',
                'tu': 'TURN',
                'ad':'ADJUST',
                'j': 'JUMP',
                'c': 'CRAWL',
                'f': 'FORCECONTROL',
                'te': 'TEST',
                'i':'IDLE',
                'u':'UPSLOPE',
                'z':'ZERO',
                'w':'WAITING'
                #  加入新步态 'a':'AAA',
                }
rospy.init_node('head_commander',anonymous=True)
command_pubber = rospy.Publisher('super_minitaur/state_change',String,queue_size=10)

print("Waiting for Odrive")
while rospy.get_param('Odrive_ready_flag') == 0:
    time.sleep(0.1)
print("Odrive is Ready")

while not rospy.is_shutdown():
    rospy.logwarn("********* Please Input Command to Change Working State **********")
    rospy.logwarn("'s' to SQUAT")
    rospy.logwarn("'t' to TROT")
    rospy.logwarn("'tu' to TURN")
    rospy.logwarn("'t4' to TRIAL4")
    rospy.logwarn("'ad' to ADJUST")
    rospy.logwarn("'j' to JUMP")
    rospy.logwarn("'c' to CRAWL")
    rospy.logwarn("'f' to FORCECONTROL")
    rospy.logwarn("'w' to 'WAITING'")
    #加入rospy.logwarn("'a' to AAA")
    command = raw_input(":")
    if command in command_dict:
        command_pubber.publish(String(command_dict[command]))
    elif command == 'yang':
        command_pubber.publish('TEST')
        time.sleep(3)
        command_pubber.publish('JUMP')
        time.sleep(3)
        command_pubber.publish('TRIAL4')
        time.sleep(3)
        command_pubber.publish('JUMP')
        time.sleep(3)
        command_pubber.publish('ADJUST')
        time.sleep(1)
    elif command == 'q':
        exit()
    else:
        rospy.logwarn('illegal input!')


