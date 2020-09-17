import rospy
import message_filters as mf
from std_msgs.msg import String
from autoware_msgs.msg import LaneArray, Lane, LaneInfo
from geometry_msgs.msg import PoseStamped

#from rospy_tutorials.msg import Floats
from std_msgs.msg import Float32MultiArray as Floats
from rospy.numpy_msg import numpy_msg

import numpy as np

global pub_best_rollout

pub_local_cost = rospy.Publisher('local_trajectory_cost', Lane, queue_size=10)
pub_best_rollout = rospy.Publisher('best_rollout',String,queue_size=10)

# 1:Need to set target lane, 0:Target lane exists
change_done=1
target_lane=-1
i=0

def callback(info):

	global change_done
	global target_lane
	global change
	global i

	# Compute change
	if change_done==1:
	
		if i==0:
			change=-1
			i+=1
		elif i<10:
			change=0
			i+=1
		elif i==10:
			change=1
			i+=1
		else:
			change=0

	# Compute rollout index
	if info.current_lane==1 or info.current_lane==info.total_lane:
		left_rollout=0
		right_rollout=1
	else:
		left_rollout=0
		right_rollout=2

	if info.current_lane==1:
		current_rollout=0
	else:
		current_rollout=1


	if change==0:
		# Receive current lane signal
		change_done=1
		target_lane=info.current_lane
		target_rollout=current_rollout
	else: 
		# Receive left/right lane change signal

		# Set target lane
		if change_done==1:
			target_lane=info.current_lane+change
			change_done=0

		# Error		
		if target_lane<0 or target_lane>info.total_lane:
			print("Error")
			# Need to reset target lane
			change_done=1
			target_rollout=current_rollout
			
		else:
			# Check lane change
			if target_lane==info.current_lane:
				change_done=1
				target_rollout=current_rollout
			else:
				target_rollout= left_rollout if change==-1 else right_rollout
		

	print(target_lane,target_rollout)

	lane=Lane(closest_object_distance=0,closest_object_velocity=0,cost=0,lane_index=target_rollout)
	pub_local_cost.publish(lane)

def listener():
	rospy.init_node('best_rollout')
	lane_info = mf.Subscriber('/lane_info', LaneInfo)
	#lane_change = mf.Subscriber('/need_to_change',String)

	ts = mf.ApproximateTimeSynchronizer([lane_info], queue_size=10, slop=0.1, allow_headerless=True)
	ts.registerCallback(callback)
	rospy.spin()


if __name__=='__main__':
	listener()
