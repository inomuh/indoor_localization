#!/usr/bin/env python

import unittest
import math

from shapely.geometry import Point, Polygon

import indoor_localization.kpi_calculation_node as kcn

from indoor_localization.msg import AnchorScan
from indoor_localization.msg import PositionInfo
from indoor_localization.msg import ParamsKPI

PKG = 'indoor_localization'
NAME = 'kpi_calculation_node_test'

class TestKPICalculationNode(unittest.TestCase):

    regions_dict = {
        'R0': [[105.0, 80.0, 10.0],
               [160.0, 80.0, 10.0],
               [160.0, 160.0, 10.0],
               [105.0, 160.0, 10.0]],
        'R1': [[80.0, 0.0, 10.0],
               [120.0, 0.0, 10.0],
               [120.0, 40.0, 10.0],
               [80.0, 40.0, 10.0]],
        'R2': [[20.0, 60.0, 10.0],
               [40.0, 60.0, 10.0],
               [40.0, 100.0, 10.0],
               [20.0, 100.0, 10.0]]
    }

    current_pos = [100.0, 100.0, 1.1]

    pos_stack = [
        [98, 98, 1.1, 25.0],
        [98, 99, 1.1, 25.5],
        [98, 100, 1.1, 26.0],
        [98, 101, 1.1, 26.5],
        [98, 102, 1.1, 27.0],
        [98, 102, 1.1, 27.5],
        [98, 102, 1.1, 28.0],
        [98, 102, 1.1, 28.5],
        [98, 102, 1.1, 29.0],
        [98, 103, 1.1, 29.5],
        [98, 103, 1.1, 30.0]
    ]

    def test0_combine(self):
        tested = kcn.combine(30, 25)
        test_result = float(30.25)
        self.assertEqual(tested, test_result)
    
    def test1_combine(self):
        tested = kcn.combine(0, 25)
        test_result = float(0.25)
        self.assertEqual(tested, test_result)

    def test2_combine(self):
        tested = kcn.combine(12, 0)
        test_result = float(12)
        self.assertEqual(tested, test_result)

    def test0_set_new_g_kpi_dictionary(self):
        tested = kcn.set_new_g_kpi_dictionary()
        test_result = {
            'kpi1': float(0.0),
            'kpi2': float(0.0),
            'kpi3': float(0.0),
            'kpi4': float(0.0)
        }
        self.assertDictEqual(tested, test_result)
    
    def test0_set_new_rb_kpi_dictionary(self):
        region_names = self.regions_dict.keys()
        tested = kcn.set_new_rb_kpi_dictionary(region_names)
        test_result = {
            'R0':{'kpi5': float(0.0), 'kpi6': float(0.0), 'kpi7': float(0.0), 'kpi8': float(0.0), 'kpi9': float(0.0)},
            'R1':{'kpi5': float(0.0), 'kpi6': float(0.0), 'kpi7': float(0.0), 'kpi8': float(0.0), 'kpi9': float(0.0)},
            'R2':{'kpi5': float(0.0), 'kpi6': float(0.0), 'kpi7': float(0.0), 'kpi8': float(0.0), 'kpi9': float(0.0)}
        }
        self.assertDictEqual(tested, test_result)

    def test0_calc_dist_travelled(self):
        cnt = 2
        tested = kcn.calc_dist_travelled(self.pos_stack, cnt)
        test_result = 1
        self.assertAlmostEqual(tested, test_result)
    
    def test0_calc_idle_time(self):
        cnt = 6
        distance_interval = 0
        thr = 1
        tested = kcn.calc_idle_time(self.pos_stack, cnt, distance_interval, thr)
        test_result = 0.5
        self.assertEqual(tested, test_result)

    def test1_calc_idle_time(self):
        cnt = 1
        distance_interval = 1
        thr = 1
        tested = kcn.calc_idle_time(self.pos_stack, cnt, distance_interval, thr)
        test_result = 0
        self.assertEqual(tested, test_result)
    
    def test0_calc_motion_time(self):
        cnt = 2
        distance_interval = 1
        thr = 1
        tested = kcn.calc_motion_time(self.pos_stack, cnt, distance_interval, thr)
        test_result = 0.5
        self.assertEqual(tested, test_result)
    
    def test1_calc_motion_time(self):
        cnt = 6
        distance_interval = 0
        thr = 1
        tested = kcn.calc_motion_time(self.pos_stack, cnt, distance_interval, thr)
        test_result = 0
        self.assertEqual(tested, test_result)
    
    def test0_calc_total_time(self):
        cnt = 2
        tested = kcn.calc_total_time(self.pos_stack, cnt)
        test_result = 0.5
        self.assertEqual(tested, test_result)

    def test1_calc_total_time(self):
        cnt = 6
        tested = kcn.calc_total_time(self.pos_stack, cnt)
        test_result = 0.5
        self.assertEqual(tested, test_result)
    
    def test0_calc_efficiency(self):
        motion_time = 2.5
        elapsed_time = 5.5
        tested = kcn.calc_efficiency(motion_time, elapsed_time)
        test_result = 45.4545
        self.assertAlmostEqual(tested, test_result, 2)

if __name__ == '__main__':
	import rosunit
	rosunit.unitrun(PKG, NAME, TestKPICalculationNode , sysargs = None, coverage_packages = [str(PKG)])
    