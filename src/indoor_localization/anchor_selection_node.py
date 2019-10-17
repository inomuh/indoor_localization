#!/usr/bin/env python
# -- coding: utf-8 --
# license removed for brevity

from math import sqrt
from itertools import combinations

import rospy

import numpy as np
from numpy import linalg as LA

from plab_comm.msg import AnchorScan
from indoor_localization.msg import PositionInfo
from indoor_localization.msg import AnchorSelected

import indoor_localization.error_estimation_node as en

# ----------------------------
# GLOBAL VARIABLES #

IPS_DICT = dict()
LAST_POSITION = list()

CONTROL_LAST_POS = False
CONTROL_IPS = False

# ----------------------------
def localization_mode():
    """
    Get selected localization mode parameter
    Return: loc_mode
    Type  : int
    """

    loc_mode = int(rospy.get_param("/anchor_selection_node/localization_mode"))

    return loc_mode


def std_of_tdoa():
    """
    Return: Gc
    Type  : float
    """

    sig_c = float(rospy.get_param("/anchor_selection_node/sig_c"))

    return sig_c


def get_initial_position():
    """
    Return: initial_position
    Type  : list
    """

    initial_tx = float(rospy.get_param("/anchor_selection_node/initial_Tx"))
    initial_ty = float(rospy.get_param("/anchor_selection_node/initial_Ty"))
    initial_tz = float(rospy.get_param("/anchor_selection_node/initial_Tz"))
    initial_position = list([initial_tx, initial_ty, initial_tz])

    return initial_position

# ----------------------------


def callback_last_pos(cb_last_pos):
    """ Gets the last position's information from the /position topic. """

    global LAST_POSITION
    global CONTROL_LAST_POS

    LAST_POSITION = [cb_last_pos.Tx, cb_last_pos.Ty, cb_last_pos.Tz]
    CONTROL_LAST_POS = True


def callback_ips(cb_all_anch):
    """
    Gets the signal received anchors' coordinate and ID information
    from the /IPS topic.
    """

    global IPS_DICT
    global CONTROL_IPS

    IPS_DICT = {
        'AnchorID': list(cb_all_anch.AnchorID),
        'x': list(cb_all_anch.x),
        'y': list(cb_all_anch.y),
        'z': list(cb_all_anch.z),
        'tdoa_of_anchors': list(cb_all_anch.tdoa_of_anchors)
    }

    CONTROL_IPS = True


# ----------------------------
def ind_of_tag(position_list):          # position information must be last position
    """
    Return: tag_index
    Type  : array
    """

    tag_index = np.array(position_list)

    return tag_index


def ind_of_anch(tmp_ips_dict, row):     # IPS
    """
    Takes information of anchors.
    Return: anchor_index
    Type  : array
    """

    anchor_index = np.zeros((row, 3), dtype=float)      # size: rowx3

    list_x = list(tmp_ips_dict['x'])
    list_y = list(tmp_ips_dict['y'])
    list_z = list(tmp_ips_dict['z'])

    for i in range(row):
        anchor_index[i][0] = list_x[i]
        anchor_index[i][1] = list_y[i]
        anchor_index[i][2] = list_z[i]

    return anchor_index
# ----------------------------


def anch_tag_distance(row, tag_index, anchor_index):
    """
    Calculates the distance between tag and anchors
    Return: tag_anchor_distance_calcu
    Type  : array
    """

    tag_anchor_distance_calcu = np.zeros((row, 3), dtype=float)

    for i in range(row):
        tag_anchor_distance_calcu[i][0] = abs(tag_index[0] - anchor_index[i][0])
        tag_anchor_distance_calcu[i][1] = abs(tag_index[1] - anchor_index[i][1])
        tag_anchor_distance_calcu[i][2] = sqrt(
            pow(tag_anchor_distance_calcu[i][0], 2) +
            pow(tag_anchor_distance_calcu[i][1], 2)
            )

    return tag_anchor_distance_calcu


def ind_of_anch_tag_distance(row, tag_anchor_distance_calcu, anchor_index):
    """
    Coordinates of anchors and distance between tag and anchors are indexed
    Return: tag_anchor_index_distance
    Type  : array
    """

    tag_anchor_index_distance = np.zeros((row, 4), dtype=float)

    for i in range(row):
        for j in range(4):
            if j == 3:
                tag_anchor_index_distance[i][j] = tag_anchor_distance_calcu[i][j-1]
            else:
                tag_anchor_index_distance[i][j] = anchor_index[i][j]

    return tag_anchor_index_distance


def sorted_anch(row, tag_anchor_index_distance):
    """
    Anchors are sorted by distance
    Return: tag_anchor_sorting
    Type  : array

    """

    tag_anchor_sorting = np.zeros((row, 4), dtype=float)
    # sort by last column
    tag_anchor_sorting = tag_anchor_index_distance[tag_anchor_index_distance[:, 3].argsort()]

    return tag_anchor_sorting


def listed_anch(row, tag_anchor_sorting):
    """
    Get the list of anchors from which signal was received.
    Return: selected_anchors
    Type  : array
    """

    selected_anchors = np.zeros((row, 5), dtype=float)

    for i in range(row):
        for j in range(5):
            if j == 4:
                selected_anchors[i][j] = i
            else:
                selected_anchors[i][j] = tag_anchor_sorting[i][j]

    return selected_anchors


def anch_combination(row):
    """
    Preparation of data for PDOP function
    Triple combination of all anchors
    Return: comb_anch
    Type  : array
    """

    anchor_count = np.zeros((row), dtype=int)

    for i in range(row):
        anchor_count[i] = i

    # Get the triple combination of IDs.
    comb_anch_list = list(combinations(anchor_count, 3))
    comb_anch = np.array(comb_anch_list)

    return comb_anch


def tmp_anch_combination(row, comb_anch, selected_anchors):
    """
    Constructs the triple combinations of the list of anchors which signal was
    received.

    Return: comb_anch_tmp
    Type  : array
    """

    comb_anch_tmp = np.zeros((len(comb_anch), 14), dtype=float)

    i = 0
    while i < len(comb_anch):
        for j in range(row):
            if int(selected_anchors[j, 4]) == comb_anch[i, 0]:
                comb_anch_tmp[i, :3] = comb_anch[i, :]
                comb_anch_tmp[i, 4:7] = selected_anchors[j, :3]

            elif selected_anchors[j][4] == comb_anch[i][1]:
                comb_anch_tmp[i, :3] = comb_anch[i, :]
                comb_anch_tmp[i, 7:10] = selected_anchors[j, :3]

            elif selected_anchors[j][4] == comb_anch[i][2]:
                comb_anch_tmp[i, :3] = comb_anch[i, :]
                comb_anch_tmp[i, 10:13] = selected_anchors[j, :3]
        i = i + 1

    return comb_anch_tmp


def all_combinations(position_list, comb_anch_tmp, sig_c):
    """
    Return: comb_anch_last
    Type  : array
    """
    tag = position_list
    for count in range(len(comb_anch_tmp)):
        anch_a = [comb_anch_tmp[count, 4], comb_anch_tmp[count, 5], comb_anch_tmp[count, 6]]
        anch_b = [comb_anch_tmp[count, 7], comb_anch_tmp[count, 8], comb_anch_tmp[count, 9]]
        anch_s = [comb_anch_tmp[count, 10], comb_anch_tmp[count, 11], comb_anch_tmp[count, 12]]
        # sig_c = 0.0625
        # drms = calc_accuracy(tag, anch_a, anch_b, anch_s, sig_c)
        drms = en.calc_accuracy(tag, anch_a, anch_b, anch_s, sig_c)
        if drms == -1:
            continue
        else:
            comb_anch_tmp[count, 13] = drms

    comb_anch_last = comb_anch_tmp

    return comb_anch_last


def find_min_pdop(comb_anch_last):
    """
    Finds the minimum PDOP value
    Return: min_pdop
    Type  : float
    """

    min_pdop = min(comb_anch_last[:, 13])

    return min_pdop


def select_anchors_main(tmp_ips_dict, min_pdop, comb_anch_last):       # IPS
    """
    Return: total_anch_coord_pdop_dict
    Type  : dict
    """

    total_anch_coord_pdop_dict = dict()

    # temp_data_x_list = list(tmp_ips_dict['x'])    # UNUSED    # list of anchors' x coordinates
    # temp_data_y_list = list(tmp_ips_dict['y'])    # UNUSED    # list of anchors' y coordinates
    # temp_data_z_list = list(tmp_ips_dict['z'])    # UNUSED    # list of anchors' y coordinates

    for i in range(len(comb_anch_last)):
        if comb_anch_last[i][13] == min_pdop:

            selected_anch1_list = list()
            selected_anch2_list = list()
            selected_anch3_list = list()

            selected_anch1_list.append(comb_anch_last[i][4])    # anchor1 x
            selected_anch1_list.append(comb_anch_last[i][5])    # anchor1 y
            selected_anch1_list.append(comb_anch_last[i][6])    # anchor1 z

            selected_anch2_list.append(comb_anch_last[i][7])    # anchor2 x
            selected_anch2_list.append(comb_anch_last[i][8])    # anchor2 y
            selected_anch2_list.append(comb_anch_last[i][9])    # anchor2 z

            selected_anch3_list.append(comb_anch_last[i][10])   # anchor3 x
            selected_anch3_list.append(comb_anch_last[i][11])   # anchor3 y
            selected_anch3_list.append(comb_anch_last[i][12])   # anchor3 z

    all_ips_anchor_dict = dict()
    all_id_list = list(tmp_ips_dict['AnchorID'])
    x_list = list(tmp_ips_dict['x'])
    y_list = list(tmp_ips_dict['y'])
    z_list = list(tmp_ips_dict['z'])

    for cnt in range(len(all_id_list)):

        coord_list = list()

        coord_list.append((x_list[cnt]))
        coord_list.append((y_list[cnt]))
        coord_list.append((z_list[cnt]))

        tmp_id = all_id_list[cnt]

        all_ips_anchor_dict[tmp_id] = coord_list

    anch1_id = all_ips_anchor_dict.keys()[all_ips_anchor_dict.values().index(selected_anch1_list)]
    anch2_id = all_ips_anchor_dict.keys()[all_ips_anchor_dict.values().index(selected_anch2_list)]
    anch3_id = all_ips_anchor_dict.keys()[all_ips_anchor_dict.values().index(selected_anch3_list)]

    total_anch_coord_pdop_dict[anch1_id] = selected_anch1_list
    total_anch_coord_pdop_dict[anch2_id] = selected_anch2_list
    total_anch_coord_pdop_dict[anch3_id] = selected_anch3_list

    return total_anch_coord_pdop_dict


def find_sel_anch_index(tmp_ips_dict, selected_anchors_dict):
    """ Find the index of the selected anchors' indexes from the anchors which comes from IPS. """

    all_anchor_id = list(tmp_ips_dict['AnchorID'])
    selected_anchor_id = list(selected_anchors_dict.keys())
    sel_anch_index_list = list()

    # for i in range(len(selected_anchor_id)):
    #     sel_anch_index_list.append(all_anchor_id.index(selected_anchor_id[i]))

    for item in selected_anchor_id:
        sel_anch_index_list.append(all_anchor_id.index(item))

    return sel_anch_index_list


def subtract_one_from_each_index(sel_anch_index_list):
    """ Subtract one from the selected anchors' index values. """

    new_index_list = list()

    # for i in range(len(sel_anch_index_list)):
    #     new_index = sel_anch_index_list[i] - 1
    #     new_index_list.append(new_index)

    for item in sel_anch_index_list:
        new_index = item - 1
        new_index_list.append(new_index)

    min_ind = new_index_list.index(min(new_index_list))

    return new_index_list, min_ind


def find_the_ddoa_values(tmp_ips_dict, new_index_list):
    """
    Find the tdoa values from the tdoa values in the IPS that
    correspond to the new index.
    """

    all_tdoa = list(tmp_ips_dict['tdoa_of_anchors'])
    sel_anch_tdoa_list = list()

    for index in new_index_list:
        sel_anch_tdoa_list.append(all_tdoa[index])

    return sel_anch_tdoa_list


def detect_finalised_tdoa_values(min_ind, sel_anch_tdoa_list):
    """
    Subtract the minimum indexed selected tdoa values from the
    other tdoa values.
    """

    final_tdoa_list = list()

    ref_ddoa = sel_anch_tdoa_list[min_ind]

    for cnt in range(len(sel_anch_tdoa_list)):
        if cnt != min_ind:
            final_tdoa = sel_anch_tdoa_list[cnt] - ref_ddoa
            final_tdoa_list.append(final_tdoa)

    return final_tdoa_list


# ----------------------------


def select_anchors_except_2d(tmp_ips_dict, mode):
    """
    If the selected localization mode is not 2D, anchor selection
    is done under this module.
    """
    selected_id = list()
    selected_anchors_dict = dict()

    anchor_id_list = list(tmp_ips_dict['AnchorID'])
    x_list = list(tmp_ips_dict['x'])
    y_list = list(tmp_ips_dict['y'])
    z_list = list(tmp_ips_dict['z'])

    for j in range(mode + 1):
        # Receive the first (mode + 1) of all signal received anchors.
        selected_id.append(anchor_id_list[j])

        selected_coords = list()

        selected_coords.append(x_list[j])
        selected_coords.append(y_list[j])
        selected_coords.append(z_list[j])

        selected_anchors_dict[selected_id[j]] = selected_coords

    return selected_anchors_dict


def generate_selected_tdoa(selected_anchors_dict, mode, tag_index):
    """
    Return: tdoa_list
    Type  : list
    """

    tmp_t = tag_index

    tdoa_list = list()

    selected_id = list(selected_anchors_dict.keys())
    selected_coords = list(selected_anchors_dict.values())

    min_ind = selected_id.index(min(selected_id))

    tmp_s = np.array(selected_coords[min_ind])
    radius_s = LA.norm(tmp_t-tmp_s)

    if mode == 1:
        a_control = False

    elif mode == 2:
        a_control = False
        b_control = False

    elif mode == 3:
        a_control = False
        b_control = False
        c_control = False

    for i in range(len(selected_id)):
        if i != min_ind and not a_control:      # == False:
            tmp_a = np.array(selected_coords[i])
            radius_a = LA.norm(tmp_t-tmp_a)
            das = radius_a - radius_s
            tdoa_list.append(das)
            a_control = True

        elif i != min_ind and not b_control:        # == False:
            tmp_b = np.array(selected_coords[i])
            radius_b = LA.norm(tmp_t - tmp_b)
            dbs = radius_b - radius_s
            tdoa_list.append(dbs)
            b_control = True

        elif i != min_ind and not c_control:        # == False:
            tmp_c = np.array(selected_coords[i])
            radius_c = LA.norm(tmp_t - tmp_c)
            dcs = radius_c - radius_s
            tdoa_list.append(dcs)
            c_control = True

    return tdoa_list


# ----------------------------


def anchor_pub_sub():

    rospy.init_node('anchor_selection_node', anonymous=True)
    rospy.Subscriber('IPS', AnchorScan, callback_ips)
    rospy.Subscriber('position', PositionInfo, callback_last_pos)
    pub = rospy.Publisher('selected_anchors', AnchorSelected, queue_size=2)

    rate = rospy.Rate(int(rospy.get_param("/kpi_calculation_node/rate")))

    mode = localization_mode()
    initial_position = get_initial_position()
    sig_c = std_of_tdoa()

    while not rospy.is_shutdown():

        if CONTROL_IPS:

            all_tdoa = list(IPS_DICT['tdoa_of_anchors'])

            if all_tdoa:

                # print("\t\n\n DATA GELDI")
                # print("\t" + str(IPS_DICT))
               
                row = len(IPS_DICT['AnchorID'])
                msg = AnchorSelected()

                if CONTROL_LAST_POS:
                    position_list = LAST_POSITION
                else:
                    position_list = initial_position

                if mode == 1 or mode == 3:
                    tag_index = ind_of_tag(position_list)
                    selected_anchors_dict = select_anchors_except_2d(IPS_DICT, mode)
                    selected_tdoa = generate_selected_tdoa(selected_anchors_dict, mode, tag_index)

                elif mode == 2:
                    tag_index = ind_of_tag(position_list)
                    anchor_index = ind_of_anch(IPS_DICT, row)
                    tag_anchor_distance_calcu = anch_tag_distance(row, tag_index, anchor_index)
                    tag_anchor_index_distance = ind_of_anch_tag_distance(row, tag_anchor_distance_calcu, anchor_index)
                    tag_anchor_sorting = sorted_anch(row, tag_anchor_index_distance)
                    selected_anchors = listed_anch(row, tag_anchor_sorting)
                    comb_anch = anch_combination(row)
                    comb_anch_tmp = tmp_anch_combination(row, comb_anch, selected_anchors)
                    comb_anch_last = all_combinations(position_list, comb_anch_tmp, sig_c)
                    min_pdop = find_min_pdop(comb_anch_last)
                    selected_anchors_dict = select_anchors_main(IPS_DICT, min_pdop, comb_anch_last)

                    # selected_tdoa = generate_selected_tdoa(
                    #     selected_anchors_dict,
                    #     mode,
                    #     tag_index
                    # )

                    # STARTS FROM HERE

                    all_anc_ips_id = list(IPS_DICT['AnchorID'])
                    selected_anc_id = list(selected_anchors_dict.keys())

                    sel_anch_index_list = find_sel_anch_index(IPS_DICT, selected_anchors_dict)

                    # If min(IPS.ID) is in the list(selected_anchors.ID):
                    if min(all_anc_ips_id) in selected_anc_id:

                        # If min(IPS.ID) is at the top of the list(IPS.ID):
                        if all_anc_ips_id.index(min(all_anc_ips_id)) == 0:
                            min_id_ind_in_sel_anch = sel_anch_index_list.index(min(sel_anch_index_list))
                            sel_anch_index_list.pop(min_id_ind_in_sel_anch)
                            new_index_list, min_ind = subtract_one_from_each_index(sel_anch_index_list)
                            sel_anch_tdoa_list = find_the_ddoa_values(IPS_DICT, new_index_list)
                            selected_tdoa = sel_anch_tdoa_list

                        # If min(IPS.ID) is at the end of the list(IPS.ID):
                        elif all_anc_ips_id.index(min(all_anc_ips_id)) == row - 1:
                            max_id_ind_in_sel_anch = sel_anch_index_list.index(max(sel_anch_index_list))
                            sel_anch_index_list.pop(max_id_ind_in_sel_anch)
                            new_index_list = sel_anch_index_list
                            sel_anch_tdoa_list = find_the_ddoa_values(IPS_DICT, new_index_list)
                            selected_tdoa = sel_anch_tdoa_list

                        # If min(IPS.ID) is neither at the beginning nor end of the list(IPS.ID):
                        else:
                            new_index_list = list()
                            min_id_ind = all_anc_ips_id.index(min(all_anc_ips_id))
                            tmp_ind = sel_anch_index_list.index(min_id_ind)
                            sel_anch_index_list.pop(tmp_ind)

                            for item in sel_anch_index_list:
                                if item < min_id_ind:
                                    new_index_list.append(item)
                                else:
                                    new_index_list.append(item - 1)

                            sel_anch_tdoa_list = find_the_ddoa_values(IPS_DICT, new_index_list)
                            selected_tdoa = sel_anch_tdoa_list

                    # If min(IPS.ID) is not in the list(selected_anchors.ID):
                    else:

                        # If min(IPS.ID) is at the top of the list(IPS.ID):
                        if all_anc_ips_id.index(min(all_anc_ips_id)) == 0:
                            new_index_list, min_ind = subtract_one_from_each_index(sel_anch_index_list)
                            sel_anch_tdoa_list = find_the_ddoa_values(IPS_DICT, new_index_list)
                            selected_tdoa = detect_finalised_tdoa_values(min_ind, sel_anch_tdoa_list)

                        # If min(IPS.ID) is at the end of the list(IPS.ID):
                        elif all_anc_ips_id.index(min(all_anc_ips_id)) == row - 1:
                            sel_anch_tdoa_list = find_the_ddoa_values(IPS_DICT, sel_anch_index_list)
                            selected_tdoa = detect_finalised_tdoa_values(min_ind, sel_anch_tdoa_list)

                        # If min(IPS.ID) is neither at the beginning nor end of the list(IPS.ID):
                        else:
                            new_index_list = list()
                            min_id_ind = all_anc_ips_id.index(min(all_anc_ips_id))

                            for item in sel_anch_index_list:
                                if item < min_id_ind:
                                    new_index_list.append(item)
                                else:
                                    new_index_list.append(item - 1)

                            sel_anch_tdoa_list = find_the_ddoa_values(IPS_DICT, new_index_list)
                            selected_tdoa = detect_finalised_tdoa_values(min_ind, sel_anch_tdoa_list)

                # print("Selected anchors: " + str(selected_anchors_dict))
                # print("Selected TDOA: " + str(selected_tdoa))

                selected_keys = list(selected_anchors_dict.keys())
                selected_coords = list(selected_anchors_dict.values())

                tmp_x = list()
                tmp_y = list()
                tmp_z = list()

                for i in range(len(selected_keys)):
                    tmp_x.append(selected_coords[i][0])
                    tmp_y.append(selected_coords[i][1])
                    tmp_z.append(selected_coords[i][2])

                msg.header.stamp = rospy.Time.now()
                msg.tdoa_of_anchors = selected_tdoa
                msg.AnchorID = list(selected_keys)
                msg.x = tmp_x
                msg.y = tmp_y
                msg.z = tmp_z

            pub.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        anchor_pub_sub()
    except rospy.ROSInterruptException:
        pass
