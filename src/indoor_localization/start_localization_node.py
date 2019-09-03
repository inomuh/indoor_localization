#!/usr/bin/env python
# coding=utf-8

import rospy
import os


def main():
    rospy.init_node('start_localization_node')
    mode = int(rospy.get_param("/start_localization_node/localization_mode"))

    if mode == 2:
        print("\n\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n\n")
        print("Mevcut Mode = " + str(mode))
        print("\n\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n\n")
        os.system("roslaunch indoor_localization localization_2D.launch")
    else:
        print("\n\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n\n")
        print("Mevcut Mode = " + str(mode))
        print("\n\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n\n")
        os.system("roslaunch indoor_localization localization_other.launch")

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
