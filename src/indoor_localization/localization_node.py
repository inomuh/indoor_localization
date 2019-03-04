#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import String
from indoor_localization.msg import AnchorScan


class CalcPos2D3AItle:
	
	def __init__(self, SX, SY, SZ, AX, AY, AZ, BX, BY, BZ, TZ, dist_diff_A_S, dist_diff_B_S):
		
		iter_num = 0
		loop_run = 1
		
		TX = ( SX + AX + BX ) / 3
		TY = ( SY + AY + BY) / 3 
		
		while loop_run == 1:
			
			iter_num = iter_num + 1 
			
			rS = math.sqrt(pow((SX - TX), 2) + pow((SY - TY), 2) + pow((SZ - TZ), 2)) 
			rA = rS + dist_diff_A_S 
			rB = rS + dist_diff_B_S 

			M11 = 2.0 * (SX - AX) 
			M12 = 2.0 * (SY - AY) 

			M21 = 2.0 * (SX - BX) 
			M22 = 2.0 * (SY - BY) 
			
			N1 = pow(rA, 2) - pow(rS, 2) - 2 * (TZ - SZ) * (SZ - AZ) - pow((SX - AX), 2) - pow((SY - AY), 2) - pow((SZ - AZ), 2) + 2 * SX * (SX - AX) + 2 * SY *(SY - AY)
			N2 = pow(rB, 2) - pow(rS, 2) - 2 * (TZ - SZ) * (SZ - BZ) - pow((SX - BX), 2) - pow((SY - BY), 2) - pow((SZ - BZ), 2) + 2 * SX * (SX - BX) + 2 * SY *(SY - BY) 

			pre_TY = (N2 - (M21 * N1 / M11)) / (M22 - (M21 * M12 / M11)) 
			pre_TX = ((-1 * M21 * N1 / M11) + (M21 * M12 / M11) * pre_TY) / (-1 * M21) 

			if ((iter_num > 20) or ( math.sqrt(pow((pre_TX - TX), 2) + pow((pre_TY - TY), 2)) < 0.05)):
				loop_run = 0
			
			TX = pre_TX
			TY = pre_TY 

		self.TX_CM = (int)(TX * 100) 
		self.TY_CM = (int)(TY * 100) 
		self.TZ_CM = (int)(TZ * 100) 


def localization_node():
	
	AX = rospy.get_param("/localization_node/AX")
	AY = rospy.get_param("/localization_node/AY")
	AZ = rospy.get_param("/localization_node/AZ")
	
	BX = rospy.get_param("/localization_node/BX")
	BY = rospy.get_param("/localization_node/BX")
	BZ = rospy.get_param("/localization_node/BX")
	
	SX = rospy.get_param("/localization_node/SX")
	SY = rospy.get_param("/localization_node/SX")
	SZ = rospy.get_param("/localization_node/SX")
	
	TZ = rospy.get_param("/localization_node/TZ")
	
	# TDOA values
	
	dist_diff_A_S = rospy.get_param("/localization_node/dist_diff_A_S")
	dist_diff_B_S = rospy.get_param("/localization_node/dist_diff_B_S")
	
	pub = rospy.Publisher('IPS', AnchorScan, queue_size=10)
	rospy.init_node('localization_node', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	
	tagpos = CalcPos2D3AItle(SX, SY, SZ, AX, AY, AZ, BX, BY, BZ, TZ, dist_diff_A_S, dist_diff_B_S)
	
	while not rospy.is_shutdown():
		
		pub_info = "\nTX = %f \nTY = %f \nTZ = %f \n\n" % (tagpos.TX_CM,tagpos.TY_CM,tagpos.TZ_CM)
		rospy.loginfo(pub_info)
		pub.publish(tagpos.TX_CM,tagpos.TY_CM,tagpos.TZ_CM)
		rate.sleep()

if __name__ == '__main__':
	try:
		localization_node()
	except rospy.ROSInterruptException:
		pass

