#!/usr/bin/env python
# -- coding: utf-8 --
# license removed for brevity

import csv
import rospy
from indoor_localization.msg import AnchorScan


def init_anchors():

    anchor_info = {
        0: [12.28, 0.015, 2.34],
        1: [-0.03, -0.03, 2.245],
        2: [-0.02, 4.46, 2.29],
        3: [12.27, 4.58, 2.265] 
    }

    return anchor_info


def plab_data_publisher():

    rospy.init_node("plab_data_publish_node", anonymous = True)
    pub = rospy.Publisher('IPS', AnchorScan, queue_size=2)
    rate = rospy.Rate(25)

    readCSV = csv.DictReader(open('/home/elcin/rosin_ws/src/indoor_localization/data/plab_encoder_data_2.csv'))
    
    das_list = list()
    dbs_list = list()
    dcs_list = list()
    sec_list = list()
    nsec_list = list()

    for row in readCSV:
        das_list.append(row['DAS'])
        dbs_list.append(row['DBS'])
        dcs_list.append(row['DCS'])
        sec_list.append(row[' sec'])
        nsec_list.append(row['nsec'])

    plab_data_dict = {
        'sec': sec_list,
        'nsec': nsec_list,
        'DAS': das_list,
        'DBS': dbs_list,
        'DCS': dcs_list
    }

    anchor_info = init_anchors()
    anchor_id = list(anchor_info.keys())
    anchor_coords = list(anchor_info.values())

    anchor_coords_x = list()
    anchor_coords_y = list()
    anchor_coords_z = list()

    for item in anchor_coords:
        anchor_coords_x.append(item[0])
        anchor_coords_y.append(item[1])
        anchor_coords_z.append(item[2])
    
    # print(anchor_id)
    # print(anchor_coords_x)
    # print(anchor_coords_y)
    # print(anchor_coords_z)


    cnt = 0
    while not rospy.is_shutdown():

        msg = AnchorScan()

        msg.header.stamp.secs = int(plab_data_dict['sec'][cnt])
        msg.header.stamp.nsecs = int(plab_data_dict['nsec'][cnt])
        # msg.header.stamp = rospy.Time.now()
        msg.AnchorID = anchor_id
        msg.x = anchor_coords_x
        msg.y = anchor_coords_y
        msg.z = anchor_coords_z
        msg.tdoa_of_anchors = [float(plab_data_dict['DCS'][cnt])/100, float(plab_data_dict['DBS'][cnt])/100, float(plab_data_dict['DAS'][cnt])/100]

        if cnt < len(plab_data_dict['DAS']) - 1:
            cnt += 1
        else:
            print("\n\t\t END OF DATA \t\t\n")
            break

        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        # read_plab_data_csv()
        plab_data_publisher()
    except rospy.ROSInterruptException:
        pass
