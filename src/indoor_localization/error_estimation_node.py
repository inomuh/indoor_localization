#!/usr/bin/env python
# -- coding: utf-8 --
# license removed for brevity

from math import sqrt
import rospy
import numpy as np
from numpy import linalg as LA
from numpy.linalg import matrix_rank

from indoor_localization.msg import AnchorSelected
from indoor_localization.msg import PositionInfo
from indoor_localization.msg import ErrorEstimated

# ----------------------------
# GLOBAL VARIABLES #

POSITION = list()
SELECTED_ANCH_DICT = dict()
CONTROL_POSITION = False
CONTROL_SELECTED_ANCH = False

# ----------------------------


def localization_mode():
    """ Get selected localization mode parameter """

    loc_mode = int(rospy.get_param("/error_estimation_node/localization_mode"))
    return loc_mode


def std_of_tdoa():
    sig_c = float(rospy.get_param("/error_estimation_node/sig_c"))
    return sig_c

# ----------------------------


def callback_position(pos):
    """ Get last position of tag from /position topic """

    global POSITION
    global CONTROL_POSITION

    POSITION = [pos.Tx, pos.Ty, pos.Tz]
    CONTROL_POSITION = True


def callback_selected_anchors(cb_selected_anch):
    """ Get selected anchors from /selectedAnchors topic """

    global SELECTED_ANCH_DICT
    global CONTROL_SELECTED_ANCH

    SELECTED_ANCH_DICT = {
        'AnchorID': list(cb_selected_anch.AnchorID),
        'x': list(cb_selected_anch.x),
        'y': list(cb_selected_anch.y),
        'z': list(cb_selected_anch.z),
        'tdoa_of_anchors': list(cb_selected_anch.tdoa_of_anchors)
    }

    CONTROL_SELECTED_ANCH = True

# ----------------------------


def find_min_id_ind(tmp_sel_anch_dict):

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
            # anch_b_ind = i

    return anch_b   # , anch_b_ind


def calc_accuracy(tag,
                  anch_a,
                  anch_b,
                  anch_s,
                  sig_c):      # posiiton'dan last position gelmeli
    """This function calculates the DRMS(Distance Root Mean Square) value of
    position."""

    # Given coordinates
    # T = np.zeros((1,3), dtype=float)
    tag_x = tag[0]
    tag_y = tag[1]
    tag_z = tag[2]
    tmp_tag = np.array(tag)

    # A = np.zeros((1,3), dtype=float)
    a_x = anch_a[0]
    a_y = anch_a[1]
    # a_z = anch_a[2]     # UNUSED
    tmp_anch_a = np.array(anch_a)

    # B = np.zeros((1,3), dtype=float)
    b_x = anch_b[0]
    b_y = anch_b[1]
    # b_z = anch_b[2]     # UNUSED
    tmp_anch_b = np.array(anch_b)

    # S = np.zeros((1,3), dtype=float)
    s_x = anch_s[0]
    s_y = anch_s[1]
    s_z = anch_s[2]
    tmp_anch_s = np.array(anch_s)

    # see eq.(3), eq.(4) and eq.(5) in document "Annex_II_2D_Positioning"
    radius_s = LA.norm(tmp_tag-tmp_anch_s)
    radius_a = LA.norm(tmp_tag-tmp_anch_a)
    radius_b = LA.norm(tmp_tag-tmp_anch_b)

    # see eq.(1) in document "Annex_II_2D_Positioning"
    das = radius_a-radius_s
    dbs = radius_b-radius_s

    # see eq.(7) in document "Annex_IV_Accuracy Analysis"
    matrix_h = np.zeros((2, 2), dtype=float)
    matrix_h[0][0] = float(2*(a_x-s_x))
    matrix_h[0][1] = float(2*(a_y-s_y))
    matrix_h[1][0] = float(2*(b_x-s_x))
    matrix_h[1][1] = float(2*(b_y-s_y))

    # rank of the H matrix is ??controlled
    if matrix_rank(matrix_h) < 2:
        drms = float("inf")
    else:
        # see eq.(8) in document "Annex_IV_Accuracy Analysis"
        # K is pseudo inverse of matrix H
        matrix_k = np.zeros((2, 2), dtype=float)

        # K = ( H.transpose() * H ) \ ( H.transpose() )
        matrix_k = LA.pinv(matrix_h)

        k11 = matrix_k[0][0]
        k12 = matrix_k[0][1]
        k21 = matrix_k[1][0]
        k22 = matrix_k[1][1]

        # print("k11: " + str(k11))
        # print("k12: " + str(k12))
        # print("k21: " + str(k21))
        # print("k22: " + str(k22))

        anch_s = np.array([s_x, s_y, s_z])

        # initial condition and threshold value for loop
        sig_r = 0
        flag = 1
        thr = 0.5

        while flag == 1:
            # see eq.(20) in document "Annex_IV_Accuracy Analysis"
            x11 = 4*pow(das, 2)*pow(sig_r, 2) + \
                8*das*(radius_s+das)*sig_r*sig_c + \
                4*pow((radius_s+das), 2)*pow(sig_c, 2)

            x12 = 4*das*dbs*pow(sig_r, 2) + 4*das*radius_s*sig_r*sig_c + \
                8*das*dbs*sig_r*sig_c + 4*dbs*radius_s*sig_c*sig_r

            x21 = x12

            x22 = 4*pow(dbs, 2)*pow(sig_r, 2) + 8*dbs*(radius_s+dbs)*sig_r*sig_c + \
                4*pow((radius_s+dbs), 2)*pow(sig_c, 2)

            # see eq.(24), eq.(25) and eq(26) in document "Annex_IV_Accuracy Analysis"
            # sig_x = sqrt(pow(k11, 2)*x11 + 2*k11*k12*x12 + pow(k12, 2)*x22)
            # sig_y = sqrt(pow(k21, 2)*x11 + 2*k21*k22*x12 + pow(k22, 2)*x22)

            try:
                sig_x = sqrt(pow(k11, 2)*x11 + 2*k11*k12*x12 + pow(k12, 2)*x22)
            except ValueError as e:
                print("Gx ValueError")
                return -1
            # print("Gx kök içi: " + str(pow(k11, 2)*x11 + 2*k11*k12*x12 + pow(k12, 2)*x22))

            try:
                sig_y = sqrt(pow(k21, 2)*x11 + 2*k21*k22*x12 + pow(k22, 2)*x22)
            except ValueError as e:
                print("Gy ValueError")
                return -1
            # print("Gy kök içi: " + str(pow(k21, 2)*x11 + 2*k21*k22*x12 + pow(k22, 2)*x22))
            # print("\n")

            sig_x_sig_y = (k11*x11+k12*x21)*k21 + (k11*x12+k12*x22)*k22

            # see Step 9 in iterative solution in document "Annex_IV_Accuracy Analysis"
            if sig_x_sig_y >= 0:
                p2 = np.array([tag_x+sig_x/2, tag_y+sig_y/2, tag_z])
                p1 = np.array([tag_x-sig_x/2, tag_y-sig_y/2, tag_z])
            else:
                p2 = np.array([tag_x-sig_x/2, tag_y+sig_y/2, tag_z])
                p1 = np.array([tag_x+sig_x/2, tag_y-sig_y/2, tag_z])

            # new radius_s values are calculated
            radius_s_1 = LA.norm(p1-anch_s)
            radius_s_2 = LA.norm(p2-anch_s)

            # threshold value is controlled
            sig_r_old = sig_r
            sig_r = abs(radius_s_2-radius_s_1)

            if abs(sig_r_old - sig_r) < thr:
                flag = 0

        # see eq.(1) in document "Annex_IV_Accuracy Analysis"
        # drms is calculated
        drms = sqrt(pow(sig_x, 2)+pow(sig_y, 2))

    return drms


def accuracy_pub_sub():

    rospy.init_node('error_estimation_node', anonymous=True)
    rospy.Subscriber('position', PositionInfo, callback_position)
    rospy.Subscriber('selected_anchors', AnchorSelected, callback_selected_anchors)
    pub = rospy.Publisher('error', ErrorEstimated, queue_size=2)

    rate = rospy.Rate(25)
    sig_c = std_of_tdoa()

    while not rospy.is_shutdown():
        if CONTROL_SELECTED_ANCH and CONTROL_POSITION:
            msg = ErrorEstimated()

            selected_x = list(SELECTED_ANCH_DICT['x'])
            selected_y = list(SELECTED_ANCH_DICT['y'])
            selected_z = list(SELECTED_ANCH_DICT['z'])
            min_id_ind = find_min_id_ind(SELECTED_ANCH_DICT)

            anch_s = find_anchor_s(SELECTED_ANCH_DICT, min_id_ind,
                                   selected_x, selected_y, selected_z)

            anch_a, anch_a_ind = find_anchor_a(SELECTED_ANCH_DICT, min_id_ind,
                                               selected_x, selected_y, selected_z)

            anch_b = find_anchor_b(SELECTED_ANCH_DICT, min_id_ind, anch_a_ind,
                                   selected_x, selected_y, selected_z)

            drms = calc_accuracy(POSITION, anch_a, anch_b, anch_s, sig_c)

            msg.accuracy = drms
            msg.header.stamp = rospy.Time.now()
            pub.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        accuracy_pub_sub()
    except rospy.ROSInterruptException:
        pass
