#!/usr/bin/env python 
#author:buenos

import rospy 
from std_msgs.msg import UInt8
from settings import *
from command_define import *

cmd_pub = rospy.Publisher("/cmd",UInt8, queue_size=10)

def runCmd(cmdname):
	cmd_pub.publish(cipher[cmdname])

def stand():
	runCmd("stand")
	print "im stand\n"

def up_and_down():
	runCmd("up_and_down")
	print "im up_and_down\n"

def cripple():
	runCmd("cripple")
	print "im cripple\n"

functionDict = {
	's':('stand       ','let minitaur stand steadly    '),
	'u':('up_and_down ','motion loop with up and down  '),
	'c':('cripple     ','motion loop with cripple      ')
}

if __name__=="__main__":
	print "************HEAD_TEST_NODE Version%d*********"% MINITAUR_VERSION
	rospy.init_node("commander", anonymous = True)

	while True:
		print "Type command: (list)"
		print "\tnum\tcode\tfunction\tintroduction"
		for id,name in enumerate(functionDict):
			print "\t%d\t[%s]\t%s\t%s" % (id+1,name,functionDict[name][0],functionDict[name][1])
		print "please type code to run the minitaur,key_q for quit"
		key = raw_input()
		
		if key == "q":
			break
		if key in functionDict:
			print "\n\n========\n[START] %s" % functionDict[key][0]
			try:
				eval(functionDict[key][0])()
			except Exception,e:
				print "[warning]"+str(e)+"\n"
		else:
			print "[warning] invaild command"
		
