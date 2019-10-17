#!/usr/bin/env python	
# -- coding: utf-8 --	
# license removed for brevity	

#########################################################################################
#                               INSTRUCTIONS                                            #
#########################################################################################
# TODO: ASSUME THAT YOU HAVE A SYSTEM WITH 4 ANCHORS.
#
#       LET'S SAY THE IDS OF ANCHORS ARE: 
#           101, 102, 103 and 104.
#
#       AND THEIR COORDINATES (unit: meter):
#           101: [2.675, -0.09, 1.94]
#           102: [3.42, 4.58, 2.07]
#           103: [12.475, -0.015, 2.4]
#           104: [12.473, 4.543, 2.35]
#
#       THE MOST IMPORTANT THING IN HERE IS THAT; ANCHOR WITH MINIMUM ID MUST BE
#   THE "SYNCHRONIZER" OF THE SYSTEM. SO, "ANCHOR 101" MUST BE THE SYNCHRONIZER IN THAT
#   SYSTEM.
#
#       BASED ON THIS INFORMATION, TDOA VALUES OF ANCHORS SHOULD BE LIKE THAT:
#               "TDOA_102_101",
#               "TDOA_103_101" and
#               "TDOA_104_101"
#
# NOTE: THESE INFORMATIONS MENTIONED ABOVE SHOULD BE READABLE FROM YOUR UWB HARDWARE !!!
#
#       SINCE WE HAVE THE ALL INFORMATION ABOUT THE ANCHORS, LETS TALK ABOUT HOW TO PUBLISH
#   ALL OF THAT DATA THROUGH "AnchorScan.msg".
#########################################################################################

# Import essential libraries
# If exists, add additional libraries according to your driver.
import time
import rospy

from hardware_driver_pkg.msg import AnchorScan


def read_data():

    # TODO: Read your anchor information from sensors
    #       in here.
    #       Write your own function in here.



def anchor_data_publisher():

    # Initialize ros node.
    rospy.init_node("hardware_ros_driver_template", anonymous = True)

    # This line declares that your node is publishing to the IPS topic using the message type AnchorScan.
    # DO NOT CHANGE !!!
    pub = rospy.Publisher('IPS', AnchorScan, queue_size = 2)
    rate = rospy.Rate(5)

    # Get data of anchors.
    read_data()

    while not rospy.is_shutdown():

        # Create a msg object.
        msg = AnchorScan()

        # Get ROS Time in here.
        # DO NOT CHANGE !!!
        msg.header.stamp = rospy.Time.now()


        msg.AnchorID = [101, 102, 103, 104]         # IDs of anchors.
                                                    # This value must be read from the sensor.
                                                    # This is the example of the content.

        msg.x = [2.675, 3.42, 12.475, 12.473]       # x coordinates of anchors
                                                    # This value must be read from the sensor.
                                                    # This is the example of the content.
                
        msg.y = [-0.09, 4.58, -0.015, 4.543]        # y coordinates of anchors
                                                    # This value must be read from the sensor.
                                                    # This is the example of the content.
                
        msg.z = [1.94, 2.07, 2.4, 2.35]             # z coordinates of anchors
                                                    # This value must be read from the sensor.
                                                    # This is the example of the content.

        msg.tdoa_of_anchors = [TDOA_102_101, TDOA_103_101, TDOA_104_101]    # TDOA values of anchors
                                                                            # This value must be read from the sensor.
                                                                            # This is the example of the content.
        
        pub.publish(msg)	
        rate.sleep()	


if __name__ == '__main__':	
    try:		
        anchor_data_publisher()	
    except rospy.ROSInterruptException:	
        pass
