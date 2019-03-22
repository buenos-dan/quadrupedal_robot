#!/usr/bin/env python
# coding:utf-8
import rospy
from std_msgs.msg import String
import time

command_dict = {'s':'SQUAT',
                'js':'JUMP_SQUAT',
                't':'TROT',
                'tu':'TURN',
                'tus':'TROT_UP_SLIP',
                'tds':'TROT_DOWN_SLIP',
                'bus':'BOUND_UP_SLIP',
                'bds':'BOUND_DOWN_SLIP',
                'j':'JUMP',
                'pj':'POS_JUMP',
                'pju':'POS_JUMP_UP',
                'jt':'JUMP_TURN',
                'q':'QUIT'}
rospy.init_node('head_commander',anonymous=True)
command_pubber = rospy.Publisher('super_minitaur/state_change',String,queue_size=10)

rospy.logwarn("Waiting for Odrive")
while rospy.get_param('Odrive_ready_flag') == 0:
    time.sleep(0.1)
rospy.logwarn("Odrive is Ready")

while not rospy.is_shutdown():
    rospy.logwarn("********* Please Input Command to Change Working State **********")
    rospy.logwarn("'s' to SQUAT")
    rospy.logwarn("'js' to JUMP_SQUAT")
    rospy.logwarn("'t' to TROT")
    rospy.logwarn("'tu' to TURN")
    rospy.logwarn("'tus' to TROT_UP_SLIP")
    rospy.logwarn("'tds' to TROT_DOWN_SLIP")
    rospy.logwarn("'j' to JUMP")
    rospy.logwarn("'jt' to JUMP_TURN")
    rospy.logwarn("'pj' to POS_JUMP")
    rospy.logwarn("'pju' to POS_JUMP_UP")
    rospy.logwarn("'bus' to BOUND_UP_SLIP")
    rospy.logwarn("'bds' to BOUND_DOWN_SLIP")
    command = raw_input(":")
    if command in command_dict:
        command_pubber.publish(String(command_dict[command]))
    elif command == 'q':
        exit()
    else:
        rospy.logwarn('illegal input!')

