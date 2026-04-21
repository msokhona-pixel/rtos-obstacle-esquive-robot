#!/usr/bin/env python
""" ROS python pogramming with finite state machines to describe a robot's behaviors
    Seatech/SYSMER 2A Course
    free to use so long as the author and other contributers are credited.
"""
#############################################################################
# imports
#############################################################################
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from fsm import fsm
from kobuki_msgs.msg import BumperEvent
#############################################################################
# class RobotBehavior
#############################################################################
class RobotBehavior(object):
	#############################################################################
	# constructor, called at creation of instance
	#############################################################################
	def __init__(self, handle_pub, T):
		self.twist = Twist()
		self.twist_real = Twist()
		self.vreal = 0.0 # longitudial velocity
		self.wreal = 0.0 # angular velocity
		self.vmax = 1.5
		self.wmax = 4.0
		self.previous_signal = 0
		self.button_pressed = False
		self.joy_activated = False
		self.pub = handle_pub
		self.T = T
		self.bumpdetected=False
		self.bumper_no=False
		self.state_start_time=rospy.get_time()
		self.Recule=False
		self.Stop2=False
		self.Rotate=False
		self.Stop3=False
		self.AutonomousMode1=False
		# instance of fsm with source state, destination state, condition (transition), callback (defaut: None)
		self.fs = fsm([ ("Start","JoyControl", True ),
			    ("JoyControl","AutonomousMode1", self.check_JoyControl_To_AutonomousMode1, self.DoAutonomousMode1),
			    ("JoyControl","JoyControl", self.KeepJoyControl, self.DoJoyControl),
			    ("AutonomousMode1","JoyControl", self.check_AutonomousMode1_To_JoyControl, self.DoJoyControl),
			    ("AutonomousMode1","AutonomousMode1", self.KeepAutonomousMode1, self.DoAutonomousMode1),
			    ("AutonomousMode1","Stop1", self.check_AutonomousMode1_To_Stop1, self.DoStop1),
			    ("Stop1","Recule", self.check_Stop1_To_Recule, self.DoRecule),
			    ("Recule","Stop2", self.check_Recule_To_Stop2, self.DoStop2),
			    ("Stop2","Rotate", self.check_Stop2_To_Rotate, self.DoRotate),
			    ("Rotate","Stop3", self.check_Rotate_To_Stop3, self.DoStop3),
			    ("Stop3","AutonomousMode1", self.check_Stop3_To_AutonomousMode1, self.DoAutonomousMode1)])
	
	#############################################################################
	# callback for joystick feedback
	#############################################################################
	def callback(self,data):
	    	#rospy.loginfo(rospy.get_name() + ": j'ai recu %f,%f", data.axes[1],data.axes[2])
		self.twist.linear.x = self.vmax * data.axes[1]
		self.twist.linear.y = 0
		self.twist.linear.z = 0
		self.twist.angular.x = 0
		self.twist.angular.y = 0
		self.twist.angular.z = self.wmax*data.axes[2]
		#rospy.loginfo(rospy.get_name() + ": I publish linear=(%f,%f,%f), angular=(%f,%f,%f)",twist.linear.x,twist.linear.y,twist.linear.z,twist.angular.x,twist.angular.y,twist.angular.z)
	
		# for transition conditions of fsm
		if not self.button_pressed:
			self.button_pressed = (self.previous_signal==0 and data.buttons[0]==1)		
		self.previous_signal = data.buttons[0]

		self.joy_activated = (abs(data.axes[1])>0.001 or abs(data.axes[2])>0.001)


	#############################################################################
	# smoothing velocity function to avoid brutal change of velocity
	#############################################################################
	def smooth_velocity(self):
		accmax = 0.01;
		accwmax = 0.05;
		vjoy = 0.0
		wjoy = 0.0
		vold = 0.0
		wold = 0.0	

		#filter twist
		vjoy = self.twist.linear.x
		vold = self.vreal
		deltav_max = accmax / self.T
		wjoy = self.twist.angular.z
		wold = self.wreal
		deltaw_max = accwmax / self.T
	
	
		#vreal
		if abs(vjoy - self.vreal) < deltav_max:
			self.vreal = vjoy
		else:
			self.vreal+=(vjoy-self.vreal)/abs(vjoy-self.vreal)*deltav_max
		
		self.vreal = max(min(self.vreal,self.vmax), -self.vmax)
	
		#wreal
		if abs(wjoy - self.wreal) < deltaw_max:
			self.wreal = wjoy
		else:
			self.wreal+=(wjoy-self.wreal)/abs(wjoy-self.wreal)*deltaw_max
			
		self.wreal = max(min(self.wreal,self.wmax), -self.wmax)	
		self.twist_real.linear.x = self.vreal	
		self.twist_real.angular.z = self.wreal
		
	def processBump(self,data):
		if ((not self.bumpdetected) and (data.state==BumperEvent.PRESSED)):
			self.bumpdetected=True
			self.bumper_no=data.bumper
			rospy.loginfo("no.%d, val:%d", data.bumper, data.state)
	
	#############################################################################
	# functions for fsm transitions
	#############################################################################
	def check_JoyControl_To_AutonomousMode1(self,fss):
		return self.button_pressed

	def check_AutonomousMode1_To_JoyControl(self,fss):
		#return self.button_pressed
		return self.joy_activated
    
	def check_AutonomousMode1_To_Stop1(self, fss):
		return self.bumpdetected
        
	def check_Recule_To_Stop2(self, fss):
		return self.Stop2
	    
	def check_Stop1_To_Recule(self, fss):
		return self.Recule
    
	def check_Stop2_To_Rotate(self, fss):
		self.AutonomousMode1 = False
		return self.Rotate
    
	def check_Rotate_To_Stop3(self, fss):
        	return self.Stop3
		
	def check_Stop3_To_AutonomousMode1(self, fss):
		if  self.AutonomousMode1:
			self.reset_flags()
			return True
		return False
		
	def KeepJoyControl(self,fss):
		return (not self.check_JoyControl_To_AutonomousMode1(fss))
		
	def KeepAutonomousMode1(self,fss):
		return not (self.check_AutonomousMode1_To_Stop1(fss) or self.check_AutonomousMode1_To_JoyControl(fss))	

	def KeepStop1(self,fss):
   		return (not self.check_Stop1_To_Recule(fss))
   		
	def Recule(self,fss):
   		return (not self.check_Recule_To_Stop2(fss))

	def KeepStop2(self,fss):
		return (not self.check_Stop2_To_Rotate(fss))
			
	def Rotate(self,fss):
   		return (not self.check_Rotate_To_Stop3(fss))

	def KeepStop3(self,fss):
		return (not self.check_Stop3_To_AutonomousMode1(fss))
	#############################################################################
	# functions for instructions inside states of fsm
	#############################################################################
	def DoJoyControl(self,fss,value):
		self.button_pressed =  False;
		self.smooth_velocity()
		self.pub.publish(self.twist_real)
		#print ('joy control : ',self.twist_real)
		pass

	def DoAutonomousMode1(self,fss,value):
		self.button_pressed =  False;
		go_fwd = Twist()
		go_fwd.linear.x = self.vmax/4.0
		self.pub.publish(go_fwd)
		self.state_start_time = rospy.get_time()

	def DoStop1(self,fss,value):
		stop_msg=Twist()
		stop_msg.linear.x=0
		self.pub.publish(stop_msg)
		self.state_start_time=rospy.get_time()
		rospy.sleep(2)
		self.Recule=True
	
	def DoRecule(self,fss,value):
		move_back=Twist()
		move_back.linear.x=-0.5
		self.pub.publish(move_back)
		self.state_start_time=rospy.get_time()
		rospy.sleep(2)
		self.Stop2=True
		
	def DoStop2(self,fss,value):
		stop_msg=Twist()
		stop_msg.linear.x=0.0
		self.pub.publish(stop_msg)
		self.state_start_time=rospy.get_time()
		rospy.sleep(2)
		self.Rotate=True
	
	
	def DoRotate(self,fss,value):
		rotate_msg=Twist()
		rotate_msg.angular.z=self.wmax/1.0
		self.pub.publish(rotate_msg)
		self.state_start_time=rospy.get_time()
		rospy.sleep(2)
		self.Stop3=True
	
	def DoStop3(self,fss,value):
		stop_msg=Twist()
		stop_msg.linear.x=0.0
		self.pub.publish(stop_msg)
		self.state_start_time=rospy.get_time()
		rospy.sleep(2)
		self.AutonomousMode1=True

	 
	def reset_flags(self):
        	self.bumpDetected = False
        	self.Recule = False
        	self.Stop2 = False
       		self.Rotate = False
        	self.Stop3 = False
        	self.AutonomousMode1 = False

	
#############################################################################
# main function
#############################################################################
if __name__ == '__main__':
	try:

		rospy.init_node('joy4ctrl')
		# real turtlebot2
		pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=1)
		# real turtlebot3
		#pub = rospy.Publisher('cmd_vel', Twist)
		# turtlesim	
		#pub = rospy.Publisher('turtle1/cmd_vel', Twist)
		Hz = 10
		rate = rospy.Rate(Hz)
		T = 1.0/Hz

		MyRobot = RobotBehavior(pub,T);
		rospy.Subscriber("joy", Joy, MyRobot.callback, queue_size=1)
		MyRobot.fs.start("Start")
		rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, MyRobot.processBump, queue_size=1)

		# loop at rate Hz
		while (not rospy.is_shutdown()):
			ret = MyRobot.fs.event("")
			rate.sleep()

	except rospy.ROSInterruptException:
        	pass
 
