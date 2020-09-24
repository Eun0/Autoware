import rospy
import message_filters as mf
from std_msgs.msg import String
from autoware_msgs.msg import LaneArray, Lane
from geometry_msgs.msg import PoseStamped

#from rospy_tutorials.msg import Floats
from std_msgs.msg import Float32MultiArray as Floats
from rospy.numpy_msg import numpy_msg

import numpy as np

global pub_best_rollout

pub_tc = rospy.Publisher('local_trajectory_cost', Lane, queue_size=10)
pub_best_rollout = rospy.Publisher('best_rollout',String,queue_size=10)
pub_costs=rospy.Publisher('local_costs',LaneArray,queue_size=10)
pub_cost=rospy.Publisher('cost',numpy_msg(Floats),queue_size=10)

# by kimna
max_same_cnt = 3
pre_best_rollout = -1
pre_rollout=-1
same_cnt=0

def callback(data):
	# by kimna
	global pre_best_rollout
	global same_cnt
	global pre_rollout


	num_rollouts = 5
	trans_cost=5
	max_dist=20
	avoid_dist=13
	center_idx=2

	# 0:obj_dist, 1:obj_vel, 2:cost, 3:lane_index, 4:is_blocked
	costs = np.zeros((5,5))
	#print(data.data.shape)
	objects = data.data.reshape((2, num_rollouts))
	#print(objects)


	all_free = True
	best_rollout= 0
	best_cost=0

	lanes=[]

	for i,dist in enumerate(objects[1]):

		costs[i,0]=dist 
		costs[i,1]=0
		costs[i,3]=i
		costs[i,4]=0

		cost=dist if dist<max_dist else max_dist		
		costs[i,2]=cost

		if dist < avoid_dist:
			all_free = False
			costs[i,4]=1
			continue
		else:
			# Add preference
			costs[i,2]+=trans_cost-2*abs(i-center_idx)

		if best_cost<costs[i,2]:
			best_cost=cost
			best_rollout=i

		
		lanes.append(Lane(closest_object_distance=costs[i,0],closest_object_velocity=costs[i,1],cost=costs[i,2],lane_index=int(costs[i,3]),is_blocked=bool(costs[i,4])))
	print("cost :",costs[:,2])
	print("block :"costs[:,4])

	if all_free is True or (abs(costs[2,2]-costs[best_rollout,2])<1 and costs[2,4]==0):
		best_rollout=2

	if pre_best_rollout==-1:
		pre_best_rollout=best_rollout
		pre_rollout=best_rollout
	
	# by kimnass

	print(best_rollout,pre_rollout,end=' ')
	
	if best_rollout == pre_rollout:
		same_cnt += 1
		if same_cnt < max_same_cnt:
			best_rollout = pre_best_rollout
		else:
			pre_best_rollout = best_rollout
	else:
		same_cnt = 0
		pre_rollout = best_rollout
		best_rollout = pre_best_rollout
	
	print("best rollout :",best_rollout,"same count :",same_cnt)
	

	#if best_rollout == pre_best_rollout:
	#	diff_cnt = 0
	#else:
	#	if pre_rollout==best_rollout:
	#		diff_cnt +=1
	#	else:
	#		diff_cnt=0

	#	if diff_cnt < max_diff_cnt:
	#		best_rollout = pre_best_rollout
	#	else:			
	#		pre_best_rollout = best_rollout
	
	
	#best_rollout=0
	best_lane=Lane(closest_object_distance=costs[best_rollout,0],closest_object_velocity=costs[best_rollout,1],cost=costs[best_rollout,2],lane_index=int(costs[best_rollout,3]),is_blocked=bool(costs[best_rollout,4]))
	pub_tc.publish(best_lane)
	pub_best_rollout.publish(String(data=str(best_rollout)))
	pub_costs.publish(LaneArray(lanes=lanes))
	pub_cost.publish(Floats(data=costs[:,2].flatten()))

def listener():
	rospy.init_node('best_rollout')
	sub_path_min = mf.Subscriber('/path_min', numpy_msg(Floats))
	ts = mf.ApproximateTimeSynchronizer([sub_path_min], queue_size=10, slop=0.1, allow_headerless=True)
	ts.registerCallback(callback)
	rospy.spin()


if __name__=='__main__':
		listener()
