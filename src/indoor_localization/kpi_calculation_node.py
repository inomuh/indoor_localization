#!/usr/bin/env python
# -- coding: utf-8 --
# license removed for brevity

import math
import rospy

from shapely.geometry import Point, Polygon

from indoor_localization.msg import PositionInfo
from indoor_localization.msg import ParamsKPI


# ----------------------------
# GLOBAL VARIABLES #

CURRENT_POS = list()
CONTROL_POS = False

# ----------------------------


def callback_pos(cb_last_pos):

    global CURRENT_POS
    global CONTROL_POS

    CURRENT_POS = [cb_last_pos.Tx, cb_last_pos.Ty, cb_last_pos.Tz, rospy.get_time()]
    CONTROL_POS = True


# ----------------------------
def get_threshold_value():
    """
    Gets the threshold values from 'localization_params.yaml' file as a
    float number.
    """

    thr = float(rospy.get_param("kpi_calculation_node/thr"))
    return thr


def get_regions():
    """ Gets the regions from 'region_params.yaml' file as a dictionary. """

    regions = dict(rospy.get_param("kpi_calculation_node/regions"))
    return regions


def generate_polygon(tmp_regions):
    """ Returns the regions_dict as 'polygon_dict' """

    all_region_names = tmp_regions.keys()
    region_coordinates = tmp_regions.values()

    polygon_dict = dict()

    for i in range(len(all_region_names)):
        polygon_dict[all_region_names[i]] = Polygon(region_coordinates[i])

    return polygon_dict


def detect_current_region(tmp_current_pos, tmp_polygons):
    """
    Returns the region names as a string and returns is that a region or not.
    """

    polygon_names = tmp_polygons.keys()
    polygon_coordinates = tmp_polygons.values()

    current_point = Point(tmp_current_pos[0], tmp_current_pos[1])
    isregion = False

    for i in range(len(polygon_coordinates)):
        control = current_point.within(tmp_polygons[polygon_names[i]])
        if control:
            current_region_name = str(polygon_names[i])
            isregion = True

    if isregion:
        return current_region_name, isregion
    else:
        current_region_name = str("The tag is outside of defined regions!")
        return current_region_name, isregion

# ----------------------------
def calc_time_interval(tmp_pos_list, cnt):

    time_interval = float(0.0)
    time_interval = tmp_pos_list[cnt+1][3] - tmp_pos_list[cnt][3]
    return time_interval

def calc_dist_interval(tmp_pos_list, cnt):
    """ Calculates the distance interval between pre and next positions. """

    distance_interval = math.sqrt(
        pow((tmp_pos_list[cnt+1][0] - tmp_pos_list[cnt][0]), 2) +
        pow((tmp_pos_list[cnt+1][1] - tmp_pos_list[cnt][1]), 2) +
        pow((tmp_pos_list[cnt+1][2] - tmp_pos_list[cnt][2]), 2))

    return distance_interval

def calc_efficiency(motion_time, elapsed_time):
    """ It calculates the efficiency. """

    efficiency = float(((motion_time) / (elapsed_time)) * 100)
    return efficiency

def calc_total_time(tmp_pos_list, cnt):
    """ It calculates the elapsed time in a region. """

    tot_tm = tmp_pos_list[cnt+1][3] - tmp_pos_list[cnt][3]
    return tot_tm

def set_new_rb_kpi_dictionary(region_names):
    """
    Generates an empty dictionary which collects the parameters
    KPI-5, KPI-6, KPI-7, KPI-8 and KPI-9 for each region.
    It defined as 'Region Based KPI Dictionary'.
    """
    rb_kpi_dict = dict()

    for region_name in region_names:

        rb_distance_travelled = float(0.0)
        rb_total_time = float(0.0)
        rb_idle_time = float(0.0)
        rb_motion_time = float(0.0)
        rb_efficiency = float(0.0)

        rb_kpi_dict[region_name] = {
            'kpi5': rb_distance_travelled,
            'kpi6': rb_total_time,
            'kpi7': rb_idle_time,
            'kpi8': rb_motion_time,
            'kpi9': rb_efficiency
        }

    return rb_kpi_dict


def kpi_pub():

    rospy.init_node("kpi_calculation_node", anonymous=True)
    rospy.Subscriber('position', PositionInfo, callback_pos)

    pub = rospy.Publisher('kpi', ParamsKPI, queue_size=2)
    rate = rospy.Rate(int(rospy.get_param("/kpi_calculation_node/rate")))

    # GET REGIONS
    # ----------------------------
    regions_dict = get_regions()
    region_names = regions_dict.keys()
    # convert specified regions to polygons and return them as dictionary
    polygons_dict = generate_polygon(regions_dict)

    # GET THRESHOLD VAL
    # ----------------------------
    thr = get_threshold_value()

    # INITIAL VALUES
    # ----------------------------
    is_region_change = False
    temp_region_name = "TRIAL"

    pos_list = list()
    rb_pos_list = list()

    delta_dist = 0

    idle_time = 0
    motion_time = 0
    distance_travelled = 0
    efficiency = 0

    # rb_idle_time = 0
    # rb_motion_time = 0
    # rb_distance_travelled = 0
    # rb_total_time = 0
    # rb_efficiency = 0

    fifo_len = 15

    rb_kpi_dict = set_new_rb_kpi_dictionary(region_names)
    simulation_start = rospy.get_time()

    while not rospy.is_shutdown():
        msg = ParamsKPI()

        if CONTROL_POS:
# --------------------------------------------------------------------------------------

            pos_list.append(CURRENT_POS)

            # Generate FIFO
            if len(pos_list) == fifo_len:

                mean_delta_tx = 0
                mean_delta_ty = 0

                for cnt in range(len(pos_list)-1):
                    dist_tx = pos_list[cnt+1][0] - pos_list[cnt][0]
                    dist_ty = pos_list[cnt+1][1] - pos_list[cnt][1]
                    mean_delta_tx += (dist_tx / (len(pos_list) - 1))
                    mean_delta_ty += (dist_ty / (len(pos_list) - 1))

                mean_delta_dist = math.sqrt(pow(mean_delta_tx, 2) + pow(mean_delta_ty, 2))

                print(len(pos_list))
                print("\n-----MEAN DIST-----")
                print(mean_delta_dist)
                print("mean_delta_tx: " + str(mean_delta_tx))
                print("mean_delta_ty: " + str(mean_delta_ty))

                if mean_delta_dist < thr: # and len(pos_list) == (fifo_len - 1):
                    time_interval = calc_time_interval(pos_list, (fifo_len - 3))
                    idle_time += time_interval                                  # KPI 2

                elif mean_delta_dist >= thr: # and len(pos_list) == (fifo_len - 1):
                    time_interval = calc_time_interval(pos_list, (fifo_len - 3))
                    motion_time += time_interval                                # KPI 3

                    dist_interval = calc_dist_interval(pos_list, fifo_len - 3)
                    distance_travelled += dist_interval                         # KPI 1

                elapsed_time = rospy.get_time() - simulation_start
                if elapsed_time == 0:
                    pass
                else:
                    efficiency = calc_efficiency(motion_time, elapsed_time)     # KPI 4

                pos_list.pop(0)

            msg.kpi1 = distance_travelled
            msg.kpi2 = idle_time
            msg.kpi3 = motion_time
            msg.kpi4 = efficiency

# --------------------------------------------------------------------------------------

            current_region_name, isregion = detect_current_region(CURRENT_POS, polygons_dict)

            # Control whether the region is change or not.
            if current_region_name != temp_region_name:
                is_region_change = True
            else:
                is_region_change = False

            # When the region changed, re-new the stack in that region.
            if is_region_change:
                if temp_region_name not in region_names:
                    pass
                else:
                    rb_pos_list = list()
            else:
                # If the region did not changed, do nothing.
                pass

            rb_pos_list.append(CURRENT_POS)

            # Generate FIFO
            if len(rb_pos_list) == fifo_len:

                rb_mean_delta_tx = 0
                rb_mean_delta_ty = 0

                for cnt in range(len(rb_pos_list) - 1):
                    rb_dist_tx = rb_pos_list[cnt+1][0] - rb_pos_list[cnt][0]
                    rb_dist_ty = rb_pos_list[cnt+1][1] - rb_pos_list[cnt][1]
                    rb_mean_delta_tx += (rb_dist_tx / (len(rb_pos_list) - 1))
                    rb_mean_delta_ty += (rb_dist_ty / (len(rb_pos_list) - 1))

                rb_mean_delta_dist = math.sqrt(pow(rb_mean_delta_tx, 2) + pow(rb_mean_delta_ty, 2))

                if rb_mean_delta_dist < thr:
                    # RB Idle Time Calculation
                    rb_time_interval = calc_time_interval(rb_pos_list, (fifo_len - 3))
                    rb_kpi_dict[temp_region_name]['kpi7'] += rb_time_interval               # KPI 7
                    # rb_idle_time += rb_time_interval                            

                elif rb_mean_delta_dist >= thr:
                    # RB Motion Time Calculation
                    rb_time_interval = calc_time_interval(rb_pos_list, (fifo_len - 3))
                    rb_kpi_dict[temp_region_name]['kpi8'] += rb_time_interval               # KPI 8
                    # rb_motion_time += rb_time_interval

                    # RB Distance Travelled Calculation
                    rb_dist_interval = calc_dist_interval(rb_pos_list, (fifo_len - 3))
                    rb_kpi_dict[temp_region_name]['kpi5'] += rb_dist_interval               # KPI 5
                    # rb_distance_travelled += rb_dist_interval

                # RB Total Time Calculation
                rb_kpi_dict[temp_region_name]['kpi6'] += calc_total_time(rb_pos_list,       # KPI 6
                                                                         (fifo_len - 3))
                # rb_total_time += calc_total_time(rb_pos_list, (fifo_len - 3))

                if rb_kpi_dict[temp_region_name]['kpi6'] == 0:
                    rb_kpi_dict[temp_region_name]['kpi9'] = 0.0
                    # pass
                else:
                    # RB Efficiency Calculation
                    rb_kpi_dict[temp_region_name]['kpi9'] = calc_efficiency(                # KPI 9
                        rb_kpi_dict[temp_region_name]['kpi8'],
                        rb_kpi_dict[temp_region_name]['kpi6'])
                    # rb_efficiency = calc_efficiency(rb_motion_time, rb_total_time)

                # rb_kpi_dict[temp_region_name]['kpi5'] = rb_distance_travelled
                # rb_kpi_dict[temp_region_name]['kpi6'] = rb_total_time
                # rb_kpi_dict[temp_region_name]['kpi7'] = rb_idle_time
                # rb_kpi_dict[temp_region_name]['kpi8'] = rb_motion_time
                # rb_kpi_dict[temp_region_name]['kpi9'] = rb_efficiency

                rb_pos_list.pop(0)

            temp_region_name = current_region_name

            msg.current_region_name = temp_region_name

            msg.reg_1_kpi5 = rb_kpi_dict['Region1']['kpi5']
            msg.reg_1_kpi6 = rb_kpi_dict['Region1']['kpi6']
            msg.reg_1_kpi7 = rb_kpi_dict['Region1']['kpi7']
            msg.reg_1_kpi8 = rb_kpi_dict['Region1']['kpi8']
            msg.reg_1_kpi9 = rb_kpi_dict['Region1']['kpi9']

            msg.reg_2_kpi5 = rb_kpi_dict['Region2']['kpi5']
            msg.reg_2_kpi6 = rb_kpi_dict['Region2']['kpi6']
            msg.reg_2_kpi7 = rb_kpi_dict['Region2']['kpi7']
            msg.reg_2_kpi8 = rb_kpi_dict['Region2']['kpi8']
            msg.reg_2_kpi9 = rb_kpi_dict['Region2']['kpi9']

            msg.reg_3_kpi5 = rb_kpi_dict['Region3']['kpi5']
            msg.reg_3_kpi6 = rb_kpi_dict['Region3']['kpi6']
            msg.reg_3_kpi7 = rb_kpi_dict['Region3']['kpi7']
            msg.reg_3_kpi8 = rb_kpi_dict['Region3']['kpi8']
            msg.reg_3_kpi9 = rb_kpi_dict['Region3']['kpi9']

            msg.reg_4_kpi5 = rb_kpi_dict['Region4']['kpi5']
            msg.reg_4_kpi6 = rb_kpi_dict['Region4']['kpi6']
            msg.reg_4_kpi7 = rb_kpi_dict['Region4']['kpi7']
            msg.reg_4_kpi8 = rb_kpi_dict['Region4']['kpi8']
            msg.reg_4_kpi9 = rb_kpi_dict['Region4']['kpi9']

            pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        kpi_pub()
    except rospy.ROSInterruptException:
        pass
