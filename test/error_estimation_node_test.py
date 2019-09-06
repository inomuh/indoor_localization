#!/usr/bin/env python

import unittest
import math
import indoor_localization.error_estimation_node as en
from indoor_localization.msg import AnchorSelected
from indoor_localization.msg import PositionInfo
from indoor_localization.msg import ErrorEstimated

PKG = 'indoor_localization'
NAME = 'error_estimation_node_test'

class TestErrorEstimationNode(unittest.TestCase):

    # Std of TDOA values
    sig_c = 0.0625

	# Selected anchors for 2D positioning
    sel_anch_dict_0 = {
		'AnchorID': [5, 19, 13],
		'x': [8.3, 0.92, 7.79],
		'y': [4.085, 0.16, 0.96],
		'z': [1.945, 1.21, 1.64],
		'tdoa_of_anchors': [-1.089007, -4.787196]
	}

    position_0 = [2.304836, 1.378714, 1.72]

	# Selected anchors for 2D positioning
    sel_anch_dict_1 = {
		'AnchorID': [5, 19, 13],
		'x': [8.3, 0.92, 7.79],
		'y': [4.085, 0.16, 0.96],
		'z': [1.945, 1.21, 1.64],
		'tdoa_of_anchors': [-1.233632, -2.466164]
	}

    position_1 = [3.606937, 1.368999, 1.72]

# ----------------------------

    def test0_calc_accuracy(self):

        anch_a = [7.79, 0.96, 1.64]
        anch_b = [0.92, 0.16, 1.21]
        anch_s = [8.3, 4.085, 1.945]

        tested = en.calc_accuracy(self.position_0, anch_a, anch_b, anch_s, self.sig_c)
        test_result = 0.1368576

        self.assertAlmostEqual(tested, test_result, 2)


    def test1_calc_accuracy(self):

        anch_a = [7.79, 0.96, 1.64]
        anch_b = [0.92, 0.16, 1.21]
        anch_s = [8.3, 4.085, 1.945]

        tested = en.calc_accuracy(self.position_1, anch_a, anch_b, anch_s, self.sig_c)
        test_result = 0.1076753

        self.assertAlmostEqual(tested, test_result, 2)

if __name__ == '__main__':
	
	import rosunit
	# rosunit.unitrun(PKG, NAME, TestPositioningNode, sysargs = "--coverage", coverage_packages=[str(PKG)])
	rosunit.unitrun(PKG, NAME, TestErrorEstimationNode, sysargs = None, coverage_packages = None)