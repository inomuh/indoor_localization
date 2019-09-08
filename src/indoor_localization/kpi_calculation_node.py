#!/usr/bin/env python
# -- coding: utf-8 --
# license removed for brevity

import math
import rospy

from shapely.geometry import Point, Polygon

from indoor_localization.msg import AnchorScan
from indoor_localization.msg import PositionInfo
from indoor_localization.msg import ParamsKPI


# ----------------------------
# GLOBAL VARIABLES #

CURRENT_POS = list()
TIME = float()
CONTROL_POS = False
CONTROL_IPS = False

# ----------------------------


def callback_pos(cb_last_pos):

    global CURRENT_POS
    global CONTROL_POS

    pos_time = combine(
        cb_last_pos.header.stamp.secs,
        cb_last_pos.header.stamp.nsecs
        )

    CURRENT_POS = [cb_last_pos.Tx, cb_last_pos.Ty, cb_last_pos.Tz, pos_time]
    CONTROL_POS = True


def callback_ips(cb_all_anch):

    global TIME
    global CONTROL_IPS

    ips_dict = {
        'header': {
            'stamp': {
                'secs': cb_all_anch.header.stamp.secs,
                'nsecs': cb_all_anch.header.stamp.nsecs
                }
            }
    }
    time_sec = ips_dict['header']['stamp']['secs']
    time_nsec = ips_dict['header']['stamp']['nsecs']

    TIME = combine(time_sec, time_nsec)
    CONTROL_IPS = True
# ----------------------------


def combine(sec, nsec):
    """
    It combines the specified second and nanosecond values as a float number.
    """

    if nsec == 0:
        return sec

    combined_time = float(sec + nsec * 10 ** -(math.floor(math.log10(nsec)) + 1))

    return combined_time
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
        # print(isregion)
        # print(current_region_name
        return current_region_name, isregion
    else:
        current_region_name = str("The tag is outside of defined regions!")
        # print(isregion)
        # print(current_region_name)
        return current_region_name, isregion
# ----------------------------


def set_new_g_kpi_dictionary():

    kpi_dict = dict()

    distance_travelled = float(0.0)
    motion_time = float(0.0)
    idle_time = float(0.0)
    efficiency = float(0.0)

    kpi_dict = {
        'kpi1': distance_travelled,
        'kpi2': idle_time,
        'kpi3': motion_time,
        'kpi4': efficiency
    }

    return kpi_dict


def set_new_rb_kpi_dictionary(region_names):

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
# ----------------------------


def calc_dist_travelled(tmp_pos_stack, cnt):
    """ Calculates the distance interval between pre and next positions. """

    distance_interval = math.sqrt(
        pow((tmp_pos_stack[cnt+1][0] - tmp_pos_stack[cnt][0]), 2) +
        pow((tmp_pos_stack[cnt+1][1] - tmp_pos_stack[cnt][1]), 2) +
        pow((tmp_pos_stack[cnt+1][2] - tmp_pos_stack[cnt][2]), 2)
    )

    return distance_interval


def calc_idle_time(tmp_pos_stack, cnt, distance_interval, thr):
    """
    If the tag is in stationary, time intervals between pre and
    next positions are calculated.
    thr = treshold value
    """

    idle_interval = float(0.0)

    if distance_interval < thr:
        idle_interval = tmp_pos_stack[cnt+1][3] - tmp_pos_stack[cnt][3]

    return idle_interval


def calc_motion_time(tmp_pos_stack, cnt, distance_interval, thr):
    """
    If the tag is in movable stage, time intervals between pre and
    next positions are calculated.
    """

    motion_interval = float(0.0)

    if distance_interval >= thr:
        motion_interval = tmp_pos_stack[cnt+1][3] - tmp_pos_stack[cnt][3]

    return motion_interval


def calc_total_time(tmp_pos_stack, cnt):
    """ It calculates the elapsed time in a region. """

    tot_tm = tmp_pos_stack[cnt+1][3] - tmp_pos_stack[cnt][3]

    return tot_tm


def calc_efficiency(motion_time, elapsed_time):
    """ It calculates the efficiency. """

    efficiency = float(((motion_time) / (elapsed_time)) * 100)

    return efficiency


def kpi_pub():

    rospy.init_node("kpi_calculation_node", anonymous=True)
    rospy.Subscriber('position', PositionInfo, callback_pos)
    rospy.Subscriber('IPS', AnchorScan, callback_ips)
    pub = rospy.Publisher('kpi', ParamsKPI, queue_size=2)
    rate = rospy.Rate(25)

    # SET PERIOD
    # ----------------------------
    # get second and nanosecond values of period
    period_s = rospy.get_param("/kpi_calculation_node/period_sec")      # 15
    period_ns = rospy.get_param("/kpi_calculation_node/period_nsec")    # 0

    # combine the period second and nanosecond as a float number
    period = combine(period_s, period_ns)

    # kpi parametrelerinin hesaplanma aralığını belirle
    # Örn: 8 saatte bir, 15 dk'da bir ...
    kpi_calculation_period = rospy.Duration(period_s, period_ns)

    # GET REGIONS
    # ----------------------------
    # region_params.yaml'de dictionary tipinde belirlenen bölgeleri al
    regions_dict = get_regions()
    region_names = regions_dict.keys()

    # GET THRESHOLD VAL
    # ----------------------------
    thr = get_threshold_value()

    # belirlenen bölgeleri poligonlara çevir ve dictionary olarak döndür
    polygons_dict = generate_polygon(regions_dict)

    # INITIAL VALUES
    # ----------------------------
    is_region_change = False
    temp_region_name = "TRIAL"

    g_pos_stack = list()
    rb_pos_stack = list()

    g_kpi_dict = set_new_g_kpi_dictionary()
    rb_kpi_dict = set_new_rb_kpi_dictionary(region_names)

    g_cnt = 0
    rb_cnt = 0

    simulation_start = rospy.Time.now()
    # ----------------------------

    while not rospy.is_shutdown():

        msg = ParamsKPI()

        if CONTROL_POS and CONTROL_IPS:

            g_pos_stack.append(CURRENT_POS)
            rb_pos_stack.append(CURRENT_POS)

            current_region_name, isregion = detect_current_region(CURRENT_POS, polygons_dict)
            # print("\n\tREGION: " + str(current_region_name))

            # detect the region is changed or not
            if current_region_name != temp_region_name:
                is_region_change = True
                # print("\tRegion is changed.\n")
            else:
                is_region_change = False
                # print("\tRegion is same.\n")

            # As soon as the zone changes, reset pos_stack in that zone. 
            if is_region_change:
                if temp_region_name not in region_names:
                    pass
                else:
                    rb_pos_stack = list()
                    rb_cnt = 0
            else:
                pass

            if temp_region_name not in region_names:
                pass
            elif not len(rb_pos_stack) < 2:
                rb_dist_interval = calc_dist_travelled(rb_pos_stack, rb_cnt)
                rb_total_time = calc_total_time(rb_pos_stack, rb_cnt)
                rb_idle_interval = calc_idle_time(rb_pos_stack, rb_cnt, rb_dist_interval, thr)
                rb_motion_interval = calc_motion_time(rb_pos_stack, rb_cnt, rb_dist_interval, thr)

                rb_kpi_dict[temp_region_name]['kpi5'] += rb_dist_interval
                rb_kpi_dict[temp_region_name]['kpi6'] += rb_total_time
                rb_kpi_dict[temp_region_name]['kpi7'] += rb_idle_interval
                rb_kpi_dict[temp_region_name]['kpi8'] += rb_motion_interval
                rb_kpi_dict[temp_region_name]['kpi9'] = calc_efficiency(
                    rb_kpi_dict[temp_region_name]['kpi8'],
                    rb_kpi_dict[temp_region_name]['kpi6']
                    )

                rb_cnt += 1
                # print("\trb_cnt: " + str(rb_cnt))

            if not len(g_pos_stack) < 2:
                g_dist_interval = calc_dist_travelled(g_pos_stack, g_cnt)
                g_idle_interval = calc_idle_time(g_pos_stack, g_cnt, g_dist_interval, thr)
                g_motion_interval = calc_motion_time(g_pos_stack, g_cnt, g_dist_interval, thr)

                g_kpi_dict['kpi1'] += g_dist_interval
                g_kpi_dict['kpi2'] += g_idle_interval
                g_kpi_dict['kpi3'] += g_motion_interval

                g_cnt += 1
                # print("\tg_cnt: " + str(g_cnt))

            temp_region_name = current_region_name

            if rospy.Time.now() > simulation_start + kpi_calculation_period:

                g_kpi_dict['kpi4'] = calc_efficiency(g_kpi_dict['kpi3'], period)

                simulation_start = rospy.Time.now()

                print("\n\t*************************************************************************************\n")
                print("\tG Distance Travelled: " + str(g_kpi_dict['kpi1']))
                print("\tG Idle Time         : " + str(g_kpi_dict['kpi2']))
                print("\tG Motion Time       : " + str(g_kpi_dict['kpi3']))
                print("\tG Efficiency        : " + str(g_kpi_dict['kpi4']))

                print("\n\tG KPI Dict        : " + str(g_kpi_dict))

                print("\n\t__________o__________o__________o__________\n")

                print("\tRB Distance Travelled: " + str(rb_kpi_dict[temp_region_name]['kpi5']))
                print("\tRB Total Time        : " + str(rb_kpi_dict[temp_region_name]['kpi6']))
                print("\tRB Idle Time         : " + str(rb_kpi_dict[temp_region_name]['kpi7']))
                print("\tRB Motion Time       : " + str(rb_kpi_dict[temp_region_name]['kpi8']))
                print("\tRB Efficiency        : " + str(rb_kpi_dict[temp_region_name]['kpi9']))

                print("\n\tRB KPI Dict        : " + str(rb_kpi_dict))
                print("\n")

                rb_pos_stack = list()
                g_pos_stack = list()

                g_cnt = 0
                rb_cnt = 0

                rb_kpi_dict = set_new_rb_kpi_dictionary(region_names)
                g_kpi_dict = set_new_g_kpi_dictionary()

            msg.current_region = temp_region_name
            pub.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        kpi_pub()
    except rospy.ROSInterruptException:
        pass
