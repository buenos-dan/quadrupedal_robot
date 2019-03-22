# Quadrupedal Robot Framework Manual
#### __四足机器人通用框架--手册__  
关于ROS的话题，节点，消息的命名规范可转到[ROS](#ros)里查看。  
关于驱动的所需完成的功能，性能测试，注意事项可转到[Driver](#driver)里查看。  
关于机器人核心所具有的功能，可转到[Robot Kernel](#robot-kernel)里查看。  

___

# Table Content  
* [Ros Configuration](#ros)
	+ [Node](#node)
	+ [Topic](#topic)
	+ [Parameter](#param)
* [Driver](#driver)
* [Sensor](#sensor)
* [Robot Kernel](#robot-kernel)
	+ [Robot Configuration](#robot-structure)
	+ [Class LegControl](#leg-control) 
	+ [Class RobotKernel](#robot-kernel) 
___

## Ros Configuration

### Node

* __how to name a driver node?__  
	+ Odrive: "Odrive_driver".
* __how to name a sensor node?__  
	+ imu: "imu_sensor".
	+ camera: "camera_sensor".
* __how to name a robot kernel__  
	+ robot: "robot_kernel"

### Topic

* we can get __current__ form topic "driver/RF/actual_cur"  
	+  __LegCurrentMsg.msg:__   
			     `float32 first`  
			     `float32 second`  
		       `float32 third`  

* we can get __velocity__ form topic "driver/RF/actual_cur"  
	+  __LegAngleMsg.msg:__   
			     `float32 first`  
			     `float32 second`  
		         `float32 third`  

* we can get __angle__ from topic "driver/RF/actual_angle"  
	+ __LegAngleMsg.msg:__  
				  `float32 first`  
			    `float32 second`  
			    `float32 third`  

* we can set motor parameters by pubing msg to "driver/param"  
	+ __DriverParamMsg.msg:__  
			`uint8 drive_index`  
		  	`int32 pos_gain`  
		  	`float32 vel_gain`  
		  	`float32 vel_integrator_gain`  
		  	`int32 vel_limit`   
		 	`int32 current_limit`  
* we can control position,current by pubing msg to "driver/RF/control"  
	+ __LegControlMsg.msg:__  
					  `int32 control_mode`  
			 			`float32 x`  
			 			`float32 y`  
			 			`float32 z`  
	- mode=1    _position\_mode_   
	- mode=2    _current\_mode_  

### Parameter
* leg_ready_flag:
	+ `leg_RF/readyflag = 1`
___

## Driver
设计要求：  

1. 上电自动检测已连接驱动个数（最好是可活动腿的个数，三自由度需要一个半Odrive）。
然后使用ros.set_param,更改相应参数，名称参考[Ros Configuration](#ros-configuration)   
2. 检测各驱动的电压，之后每个1秒检测一次，同时更改ros_param.
3. get and sent cur vel angle value
4. receive and set control param value

___  

## Sensor 

there is a empty doc.
___
## Robot Kernel    

机器人核心被定义为一个类，其中包含了许多足式机器人所必须的最基本的功能:  
* 自检连接驱动数，自检参数配置，自检传感器种类, 保障机器人正常运行。  
* 位置控制模式下，提供足端的点对点运动，给定曲线运动类。  
* 力控制模式下，提供重力补偿，曲线跟踪，触地检测等功能。   

   >#### dependence
     >* scipy  科学计算库 
     >* matplotlib  画图库 
### Robot Configuration
* how many legs?
	+ name:RF RB LF LB
* how many motors in one leg?
	+ which decide the demensions of topic msg.  
	  3D: `first second third`   
	  2D: `0     second third`

### Class LegControl  

``` python
class LegControl:
    def __init__(self,name):
        self.name = name + "_leg"
		#states
        self.actual_cur   = np.array([0,0,0])          # [first,second,third] A
        self.actual_vel   = np.array([0,0,0])          # [first,second,third] 
        self.actual_angle = np.array([0,0,0])          # [first,second,third] rad
		#pubs 
        self.control_puber = rospy.Publisher("driver/" + name + "/control",LegControlMsg,  queue_size=10)
        self.param_puber   = rospy.Publisher("driver/parameter",           DriverParamMsg, queue_size=10)
		#subs
        self.cur_suber     = rospy.Subscriber("driver/" + name + "/actual_cur",  LegCurrentMsg, self.cur_callback  )
        self.angle_suber   = rospy.Subscriber("driver/" + name + "/actual_angle",LegAngleMsg,   self.angle_callback)
        self.vel_suber     = rospy.Subscriber("driver/" + name + "/actual_vel",  LegVelocityMsg,self.vel_callback  )

    def WritePosMsg(self,pos):
        """using driver pd control"""
        self.control_puber.publish(LegControlMsg(3,pos[0],pos[1]))

    def WriteCurMsg(self,cur):
        self.control_puber.publish(LegControlMsg(1,cur[0],cur[1]))


    def Set_para(self,pos_gain = ODRV_POS_GAIN,
                 vel_gain = ODRV_VEL_GAIN,
                 vel_integrator_gain = ODRV_VEL_INTEGRATOR_GAIN,
                 vel_limit = ODRV_VEL_LIMIT,
                 current_limit = ODRV_CURRENT_LIM):
        msg = OdriveParamMsg()
        msg.odrive_index = leg_cipher[self.leg_name][0]
        msg.pos_gain = pos_gain
        msg.vel_gain = vel_gain
        msg.vel_integrator_gain = vel_integrator_gain
        msg.vel_limit = vel_limit
        msg.current_limit = current_limit
        self.para_puber.publish(msg)

    def cur_callback(self,msg):
        self.actual_cur = [abs(msg.front_current),abs(msg.rear_current)]

    def pos_callback(self,msg):
        self.actual_pos = [msg.x,msg.y]

    def motor_angle_callback(self,msg):
        self.actual_motor_angle = [msg.x/180*pi,msg.y/180*pi]

    def calculate_jacob(self):
        jacobian = utils.Jacobian3(self.actual_motor_angle[0],self.actual_motor_angle[1])
        return jacobian

```

###Class RobotKernel
