#!/usr/bin/env python
# -- coding: utf-8 --
# license removed for brevity

from math import sqrt
import random
import rospy
from indoor_localization.msg import AnchorSelected
from indoor_localization.msg import PositionInfo

# ----------------------------
# GLOBAL VARIABLES #

SEL_ANCH_DICT = dict()
CONTROL_SEL_ANCH = False

# ----------------------------


def localization_mode():
    """
    Get selected localization mode parameter from localization_params.yaml
    """

    loc_mode = int(rospy.get_param("/positioning_node/localization_mode"))
    return loc_mode


def get_tag_z():

    tag_z = float(rospy.get_param("/positioning_node/TZ"))
    return tag_z


def start_point_of_line():
    """
    Get 'starting' points of line from localization_params.yaml

    L1: Starting point of line
    line_start_x = line_start[0]        # L1X
    line_start_y = line_start[1]        # L1Y
    line_start_z = line_start[2]        # L1Z
    """

    line_start_list = list(rospy.get_param("/positioning_node/L1"))
    return line_start_list


def end_point_of_line():
    """
    Get 'end' points of line from localization_params.yaml

    L2: End point of line
    line_end_x = line_start[0]      # L2X
    line_end_y = line_start[1]      # L2Y
    line_end_z = line_start[2]      # L2Z
    """

    line_end_list = list(rospy.get_param("/positioning_node/L2"))
    return line_end_list


# ----------------------------


def callback_selected_anchors(cb_selected_anch):
    """ Get selected anchors from /selectedAnchors topic """

    global SEL_ANCH_DICT
    global CONTROL_SEL_ANCH

    SEL_ANCH_DICT = {
        'AnchorID': list(cb_selected_anch.AnchorID),
        'x': list(cb_selected_anch.x),
        'y': list(cb_selected_anch.y),
        'z': list(cb_selected_anch.z),
        'tdoa_of_anchors': list(cb_selected_anch.tdoa_of_anchors)
    }

    CONTROL_SEL_ANCH = True


# ----------------------------


def add_noise(denominator):
    """
    This function adds some noie to the denominator to prevent the ZeroDivisionError 
    while calculating the pre_tag_z, the pre_tag_y and the pre_tag_x values.
    """

    if (denominator >= -0.01) and (denominator <= 0):
        denominator -= random.uniform(0, 0.1)
    
    elif (denominator >= 0) and (denominator <= 0.01):
        denominator += random.uniform(0, 0.1)
    
    return denominator


def find_min_id_ind(tmp_sel_anch_dict):
    """ Finds the index of minimum ID from the selected anchors' IDs. """

    selected_id = list(tmp_sel_anch_dict['AnchorID'])
    min_id_ind = selected_id.index(min(selected_id))

    return min_id_ind


def find_anchor_s(tmp_sel_anch_dict,
                  min_id_ind,
                  selected_x,
                  selected_y,
                  selected_z):

    s_x = selected_x[min_id_ind]
    s_y = selected_y[min_id_ind]
    s_z = selected_z[min_id_ind]
    anch_s = [s_x, s_y, s_z]

    return anch_s


def find_anchor_a(tmp_sel_anch_dict,
                  min_id_ind,
                  selected_x,
                  selected_y,
                  selected_z):

    for i in range(len(tmp_sel_anch_dict['AnchorID'])):
        if i != min_id_ind:
            a_x, a_y, a_z = selected_x[i], selected_y[i], selected_z[i]
            anch_a = [a_x, a_y, a_z]
            anch_a_ind = i

    return anch_a, anch_a_ind


def find_anchor_b(tmp_sel_anch_dict,
                  min_id_ind,
                  anch_a_ind,
                  selected_x,
                  selected_y,
                  selected_z):

    for i in range(len(tmp_sel_anch_dict['AnchorID'])):
        if (i != min_id_ind) and (i != anch_a_ind):
            b_x, b_y, b_z = selected_x[i], selected_y[i], selected_z[i]
            anch_b = [b_x, b_y, b_z]
            anch_b_ind = i

    return anch_b, anch_b_ind


def find_anchor_c(tmp_sel_anch_dict,
                  min_id_ind,
                  anch_a_ind,
                  anch_b_ind,
                  selected_x,
                  selected_y,
                  selected_z):

    for i in range(len(tmp_sel_anch_dict['AnchorID'])):
        if (i != min_id_ind) and (i != anch_a_ind) and (i != anch_b_ind):
            c_x, c_y, c_z = selected_x[i], selected_y[i], selected_z[i]
            anch_c = [c_x, c_y, c_z]
            # anch_c_ind = i

    return anch_c       # , anch_c_ind


def calc_dist_1d_2a_ite(anch_s, anch_a,
                        line_start, line_end,
                        dist_diff_a_s):

    """ 1D Positioning Algorithm """

    position = list()

    anch_s_x = anch_s[0]
    anch_s_y = anch_s[1]
    anch_s_z = anch_s[2]

    anch_a_x = anch_a[0]
    anch_a_y = anch_a[1]
    anch_a_z = anch_a[2]

    line_start_x = line_start[0]
    line_start_y = line_start[1]
    line_start_z = line_start[2]

    line_end_x = line_end[0]
    line_end_y = line_end[1]
    line_end_z = line_end[2]

    tag_x_new = float((line_end_x + line_start_x) / 2)
    tag_y_new = float((line_end_y + line_start_y) / 2)
    tag_z_new = float((line_end_z + line_start_z) / 2)

    N = float(
        2*(anch_a_x-anch_s_x)*line_start_x +
        2*(anch_a_y-anch_s_y)*line_start_y +
        2*(anch_a_z-anch_s_z)*line_start_z +
        (pow(anch_s_x, 2)+pow(anch_s_y, 2)+pow(anch_s_z, 2)) -
        (pow(anch_a_x, 2)+pow(anch_a_y, 2)+pow(anch_a_z, 2)) +
        pow(dist_diff_a_s, 2)
        )

    Q = float(
        2*(anch_a_x-anch_s_x)*(line_end_x-line_start_x) +
        2*(anch_a_y-anch_s_y)*(line_end_y-line_start_y) +
        2*(anch_a_z-anch_s_z)*(line_end_z-line_start_z)
        )

    iter_num = 0
    diff = 1

    while diff > 0.001:

        iter_num = iter_num + 1

        tag_x_old = tag_x_new
        tag_y_old = tag_y_new
        tag_z_old = tag_z_new

        radius_s = float(
            sqrt(
                pow((anch_s_x-tag_x_old), 2) +
                pow((anch_s_y-tag_y_old), 2) +
                pow((anch_s_z-tag_z_old), 2)
                )
            )

        m = float(-(N + (2*dist_diff_a_s*radius_s))/Q)

        tag_x_new = float(line_start_x + m*(line_end_x-line_start_x))
        tag_y_new = float(line_start_y + m*(line_end_y-line_start_y))
        tag_z_new = float(line_start_z + m*(line_end_z-line_start_z))

        diff = float(
            sqrt(
                pow((tag_x_new-tag_x_old), 2) +
                pow((tag_y_new-tag_y_old), 2) +
                pow((tag_z_new-tag_z_old), 2)
                )
            )

    distance = float(
        sqrt(
            pow((line_start_x-tag_x_new), 2) +
            pow((line_start_y-tag_y_new), 2) +
            pow((line_start_z-tag_z_new), 2)
            )
        )

    # tag_x_cm = float(tag_x_new*100)     # UNUSED
    # tag_y_cm = float(tag_y_new*100)     # UNUSED
    # tag_z_cm = float(tag_z_new*100)     # UNUSED
    # distance_cm = float(distance*100)   # UNUSED

    tag_x_m = float(tag_x_new)
    tag_y_m = float(tag_y_new)
    tag_z_m = float(tag_z_new)

    position.append(tag_x_m)
    position.append(tag_y_m)
    position.append(tag_z_m)
    position.append(distance)

    return position


def calc_pos_2d_3a_ite(anch_s, anch_a, anch_b,
                       tag_z, dist_diff_a_s, dist_diff_b_s):

    """ 2D Positioning Algorithm """

    position = list()

    iter_num = 0
    loop_run = 1

    anch_s_x = anch_s[0]
    anch_s_y = anch_s[1]
    anch_s_z = anch_s[2]

    anch_a_x = anch_a[0]
    anch_a_y = anch_a[1]
    anch_a_z = anch_a[2]

    anch_b_x = anch_b[0]
    anch_b_y = anch_b[1]
    anch_b_z = anch_b[2]

    tag_x = (anch_s_x+anch_a_x+anch_b_x)/3.0
    tag_y = (anch_s_y+anch_a_y+anch_b_y)/3.0

    while loop_run == 1:

        iter_num = iter_num + 1

        radius_s = sqrt(
            pow((anch_s_x-tag_x), 2) +
            pow((anch_s_y-tag_y), 2) +
            pow((anch_s_z-tag_z), 2)
            )
        radius_a = radius_s + dist_diff_a_s
        radius_b = radius_s + dist_diff_b_s

        # radius_a = sqrt(
        #     pow((anch_a_x-tag_x), 2) +
        #     pow((anch_a_y-tag_y), 2) +
        #     pow((anch_a_z-tag_z), 2)
        #     )
        # radius_b = sqrt(
        #     pow((anch_b_x-tag_x), 2) +
        #     pow((anch_b_y-tag_y), 2) +
        #     pow((anch_b_z-tag_z), 2)
        #     )

        M11 = 2.0 * (anch_s_x-anch_a_x)
        M12 = 2.0 * (anch_s_y-anch_a_y)
        M21 = 2.0 * (anch_s_x-anch_b_x)
        M22 = 2.0 * (anch_s_y-anch_b_y)

        N1 = float(
            pow(radius_a, 2) - pow(radius_s, 2) -
            2*(tag_z-anch_s_z)*(anch_s_z-anch_a_z) -
            pow((anch_s_x-anch_a_x), 2) -
            pow((anch_s_y-anch_a_y), 2) -
            pow((anch_s_z-anch_a_z), 2) +
            2*anch_s_x*(anch_s_x-anch_a_x) + 2*anch_s_y*(anch_s_y-anch_a_y)
            )

        N2 = float(
            pow(radius_b, 2) - pow(radius_s, 2) -
            2*(tag_z-anch_s_z)*(anch_s_z-anch_b_z) -
            pow((anch_s_x-anch_b_x), 2) -
            pow((anch_s_y-anch_b_y), 2) -
            pow((anch_s_z-anch_b_z), 2) +
            2*anch_s_x*(anch_s_x-anch_b_x) + 2*anch_s_y*(anch_s_y-anch_b_y)
            )

        if M11 == 0:
        #     M11 = float(0.1)
            M11 = add_noise(M11)

        pre_tag_y = float(
            (N2 - (M21*N1/M11))/(M22-(M21*M12/M11))
            )

        if M21 == 0:
        #     M21 = float(0.1)
            M21 = add_noise(M21)

        pre_tag_x = float(
            ((-1*M21*N1/M11) + (M21*M12/M11)*pre_tag_y)/(-1*M21)
            )

        if ((iter_num > 20) or (sqrt(pow((pre_tag_x - tag_x), 2) + pow((pre_tag_y-tag_y), 2)) < 0.05)):
            loop_run = 0

        tag_x = pre_tag_x
        tag_y = pre_tag_y

    # tag_x_cm = float(tag_x*100)     # UNUSED
    # tag_y_cm = float(tag_y*100)     # UNUSED
    # tag_z_cm = float(tag_z*100)     # UNUSED

    tag_x_m = float(tag_x)
    tag_y_m = float(tag_y)
    tag_z_m = float(tag_z)

    position.append(tag_x_m)
    position.append(tag_y_m)
    position.append(tag_z_m)

    return position


def calc_pos_3d_4a_ite(anch_s, anch_a, anch_b, anch_c,
                       dist_diff_a_s, dist_diff_b_s, dist_diff_c_s):

    """ 3D Positioning Algorithm """

    position = list()

    iter_num = 0
    loop_run = 1

    anch_s_x = anch_s[0]
    anch_s_y = anch_s[1]
    anch_s_z = anch_s[2]

    anch_a_x = anch_a[0]
    anch_a_y = anch_a[1]
    anch_a_z = anch_a[2]

    anch_b_x = anch_b[0]
    anch_b_y = anch_b[1]
    anch_b_z = anch_b[2]

    anch_c_x = anch_c[0]
    anch_c_y = anch_c[1]
    anch_c_z = anch_c[2]

    tag_x = (anch_s_x+anch_a_x+anch_b_x+anch_c_x)/4.0
    tag_y = (anch_s_y+anch_a_y+anch_b_y+anch_c_y)/4.0
    tag_z = (anch_s_z+anch_a_z+anch_b_z+anch_c_z)/4.0

    while loop_run == 1:

        iter_num = iter_num + 1

        radius_s = float(
            sqrt(
                pow((anch_s_x-tag_x), 2) +
                pow((anch_s_y-tag_y), 2) +
                pow((anch_s_z-tag_z), 2)
                )
            )
        radius_a = float(radius_s + dist_diff_a_s)
        radius_b = float(radius_s + dist_diff_b_s)
        radius_c = float(radius_s + dist_diff_c_s)

        M11 = float(2.0*(anch_s_x-anch_a_x))
        M12 = float(2.0*(anch_s_y-anch_a_y))
        M13 = float(2.0*(anch_s_z-anch_a_z))

        M21 = float(2.0*(anch_s_x-anch_b_x))
        M22 = float(2.0*(anch_s_y-anch_b_y))
        M23 = float(2.0*(anch_s_z-anch_b_z))

        M31 = float(2.0*(anch_s_x-anch_c_x))
        M32 = float(2.0*(anch_s_y-anch_c_y))
        M33 = float(2.0*(anch_s_z-anch_c_z))

        N1 = float(
            pow(radius_a, 2) - pow(radius_s, 2) +
            (pow(anch_s_x, 2)+pow(anch_s_y, 2)+pow(anch_s_z, 2)) -
            (pow(anch_a_x, 2)+pow(anch_a_y, 2)+pow(anch_a_z, 2))
            )

        N2 = float(
            pow(radius_b, 2) - pow(radius_s, 2) +
            (pow(anch_s_x, 2)+pow(anch_s_y, 2)+pow(anch_s_z, 2)) -
            (pow(anch_b_x, 2)+pow(anch_b_y, 2)+pow(anch_b_z, 2))
            )

        N3 = float(
            pow(radius_c, 2) - pow(radius_s, 2) +
            (pow(anch_s_x, 2)+pow(anch_s_y, 2)+pow(anch_s_z, 2)) -
            (pow(anch_c_x, 2)+pow(anch_c_y, 2)+pow(anch_c_z, 2))
            )
        """
        print("M11: " + str(M11))
        print("M12: " + str(M12))
        print("M13: " + str(M13))
        print("M21: " + str(M21))
        print("M22: " + str(M22))
        print("M23: " + str(M23))
        print("M31: " + str(M31))
        print("M32: " + str(M32))
        print("M33: " + str(M33))
        print("\n")
        print(((M33-(M31/M11)*M13) - (M32 - (M31/M11)*M12)*(M23-(M21/M11)*M13)/(M22-(M21/M11)*M12)))
        print((M33-(M31/M11)*M13))
        print((M32 - (M31/M11)*M12)*(M23-(M21/M11)*M13)/(M22-(M21/M11)*M12))

        print((M32 - (M31/M11)*M12))

        print((M23-(M21/M11)*M13))
        print((M22-(M21/M11)*M12))
        """

        if (M11 == 0):
            M11 = add_noise(M11)
        
        if (M12 == 0):
            M12 = add_noise(M12)

        if (M13 == 0):
            M13 = add_noise(M13)

        if (M21 == 0):
            M21 = add_noise(M21)
        
        if (M22 == 0):
            M22 = add_noise(M22)

        if (M23 == 0):
            M23 = add_noise(M23)

        if (M31 == 0):
            M31 = add_noise(M31)
        
        if (M32 == 0):
            M32 = add_noise(M32)

        if (M33 == 0):
            M33 = add_noise(M33)

        pre_tag_z = float(
            ((N3 - (M31/M11)*N1) - (M32-(M31/M11)*M12)*(N2-(M21/M11)*N1)/(M22-(M21/M11)*M12))/((M33-(M31/M11)*M13) - (M32 - (M31/M11)*M12)*(M23-(M21/M11)*M13)/(M22-(M21/M11)*M12))
            )

        pre_tag_y = float(
            ((N2-(M21/M11)*N1) - (M23-(M21/M11)*M13)*tag_z) / (M22-(M21/M11)*M12)
            )

        pre_tag_x = float((N1-M13*tag_z-M12*tag_y) / M11)

        if ((iter_num > 20) or (sqrt(pow((pre_tag_x-tag_x), 2) + pow((pre_tag_y-tag_y), 2) + pow((pre_tag_z-tag_z), 2)) < 0.05)):

            loop_run = 0

        tag_x = pre_tag_x
        tag_y = pre_tag_y
        tag_z = pre_tag_z

    # tag_x_cm = float(tag_x*100)     # UNUSED
    # tag_y_cm = float(tag_y*100)     # UNUSED
    # tag_z_cm = float(tag_z*100)     # UNUSED

    tag_x_m = float(tag_x)
    tag_y_m = float(tag_y)
    tag_z_m = float(tag_z)

    position.append(tag_x_m)
    position.append(tag_y_m)
    position.append(tag_z_m)

    return position


def position_pub_sub():

    rospy.init_node('positioning_node', anonymous=True)
    rospy.Subscriber('selected_anchors', AnchorSelected, callback_selected_anchors)
    pub = rospy.Publisher('position', PositionInfo, queue_size=2)

    rate = rospy.Rate(25)
    mode = localization_mode()

    while not rospy.is_shutdown():
        msg = PositionInfo()
        msg.header.stamp = rospy.Time.now()

        if CONTROL_SEL_ANCH:

            selected_x = list(SEL_ANCH_DICT['x'])
            selected_y = list(SEL_ANCH_DICT['y'])
            selected_z = list(SEL_ANCH_DICT['z'])
            selected_tdoa = list(SEL_ANCH_DICT['tdoa_of_anchors'])
            min_id_ind = find_min_id_ind(SEL_ANCH_DICT)

            if mode == 1:
                anch_s = find_anchor_s(SEL_ANCH_DICT,
                                       min_id_ind,
                                       selected_x,
                                       selected_y,
                                       selected_z)

                anch_a, anch_a_ind = find_anchor_a(SEL_ANCH_DICT,
                                                   min_id_ind,
                                                   selected_x,
                                                   selected_y,
                                                   selected_z)
                L1 = start_point_of_line()
                L2 = end_point_of_line()

                position = calc_dist_1d_2a_ite(anch_s, anch_a, L1, L2,
                                               selected_tdoa[0])

                msg.Tx = position[0]
                msg.Ty = position[1]
                msg.Tz = position[2]
                msg.distance = position[3]

                pub.publish(msg)

            if mode == 2:
                anch_s = find_anchor_s(SEL_ANCH_DICT,
                                       min_id_ind,
                                       selected_x,
                                       selected_y,
                                       selected_z)

                anch_a, anch_a_ind = find_anchor_a(SEL_ANCH_DICT,
                                                   min_id_ind,
                                                   selected_x,
                                                   selected_y,
                                                   selected_z)

                anch_b, anch_b_ind = find_anchor_b(SEL_ANCH_DICT,
                                                   min_id_ind,
                                                   anch_a_ind,
                                                   selected_x,
                                                   selected_y,
                                                   selected_z)
                tag_z = get_tag_z()

                position = calc_pos_2d_3a_ite(anch_s, anch_a, anch_b, tag_z,
                                              selected_tdoa[1], selected_tdoa[0])

                # print("\tHesaplanan pos: " + str(position))
                msg.Tx = position[0]
                msg.Ty = position[1]
                msg.Tz = position[2]

            pub.publish(msg)

            if mode == 3:
                anch_s = find_anchor_s(SEL_ANCH_DICT,
                                    min_id_ind,
                                    selected_x,
                                    selected_y,
                                    selected_z)

                anch_a, anch_a_ind = find_anchor_a(SEL_ANCH_DICT,
                                                min_id_ind,
                                                selected_x,
                                                selected_y,
                                                selected_z)

                anch_b, anch_b_ind = find_anchor_b(SEL_ANCH_DICT,
                                                min_id_ind,
                                                anch_a_ind,
                                                selected_x,
                                                selected_y,
                                                selected_z)

                anch_c = find_anchor_c(SEL_ANCH_DICT,
                                    min_id_ind,
                                    anch_a_ind,
                                    anch_b_ind,
                                    selected_x,
                                    selected_y,
                                    selected_z)

                position = calc_pos_3d_4a_ite(anch_s, anch_a, anch_b, anch_c,
                                            selected_tdoa[0], selected_tdoa[1], selected_tdoa[2])

                msg.Tx = position[0]
                msg.Ty = position[1]
                msg.Tz = position[2]

                pub.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        position_pub_sub()

        """
        anch_s = [2.25, 3.15, 0.11]
        anch_a = [4.21, 2.6, 2.84]
        anch_b = [4.265, 0.45, 0.11]
        anch_c = [1.84, 0.835, 2.84]
        # tag = [2.70, 1.35, 0.44]
        tag = [25, 25, 15]

        anch_s_x = anch_s[0]
        anch_s_y = anch_s[1]
        anch_s_z = anch_s[2]

        anch_a_x = anch_a[0]
        anch_a_y = anch_a[1]
        anch_a_z = anch_a[2]

        anch_b_x = anch_b[0]
        anch_b_y = anch_b[1]
        anch_b_z = anch_b[2]

        anch_c_x = anch_c[0]
        anch_c_y = anch_c[1]
        anch_c_z = anch_c[2]

        tag_x = tag[0]
        tag_y = tag[1]
        tag_z = tag[2]


        rs = sqrt(
                pow((anch_s_x-tag_x), 2) +
                pow((anch_s_y-tag_y), 2) +
                pow((anch_s_z-tag_z), 2)
                )

        ra = sqrt(
                pow((anch_a_x-tag_x), 2) +
                pow((anch_a_y-tag_y), 2) +
                pow((anch_a_z-tag_z), 2)
                )

        rb = sqrt(
                pow((anch_b_x-tag_x), 2) +
                pow((anch_b_y-tag_y), 2) +
                pow((anch_b_z-tag_z), 2)
                )

        rc = sqrt(
                pow((anch_c_x-tag_x), 2) +
                pow((anch_c_y-tag_y), 2) +
                pow((anch_c_z-tag_z), 2)
                )

        das = ra-rs
        dbs = rb-rs
        dcs = rc-rs

        pos_3d = calc_pos_3d_4a_ite(anch_s, anch_a, anch_b, anch_c, das, dbs, dcs)

        print("\n\n3D Position: " + str(pos_3d) + "\n\n")
        """


    except rospy.ROSInterruptException:
        pass
