#!/usr/bin/env python
# coding:utf-8
import rospy, time, re, serial
import numpy as np
from super_minitaur.srv import StateFlag
rospy.init_node("lower_conmunication",anonymous=True)
# rospy.wait_for_service("/state_flag_lower")
lower_client = rospy.ServiceProxy("/state_flag_lower",StateFlag)
port = serial.Serial("/dev/stm32",115200,timeout=0.5)
port.close()
port.open()
print("Lower Connected!")
old_flag = [0,0,0,0,0]
flag_change_time = [0,0,0,0,0]
get_shagai_flag = 0
while not rospy.is_shutdown():
    data = port.readline()
    match = re.search("f(\d)(\d)(\d)(\d)(\d)", data)
    if match != None:
        flag = [int(match.group(i+1)) for i in range(5)]
        changed_flag = np.array(flag) - np.array(old_flag)
        for i,change_state in enumerate(changed_flag):
            if change_state == 1:
                if time.time() - flag_change_time[i] >= 1:  # 大于一秒间隔才可发送
                    if i == 3:
                        if get_shagai_flag == 0:
                            get_shagai_flag = 1
                            lower_client.call(flag)
                            print("flag changed: {}".format(flag))
                            print("Get shagai !")
                    else:
                        lower_client.call(flag)
                        flag_change_time[i] = time.time()
                        print("flag changed: {}".format(flag))
                flag = [0,0,0,0,0]
        old_flag = flag
    time.sleep(0.1)
port.close()
