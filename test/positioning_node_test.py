#!/usr/bin/env python

import unittest
import math
import indoor_localization.positioning_node as pn
from indoor_localization.msg import AnchorSelected
from indoor_localization.msg import PositionInfo

PKG = 'indoor_localization'
NAME = 'positioning_node_test'

class TestPositioningNode(unittest.TestCase):	

	# Selected anchors for 2D positioning
	sel_anch_dict_0 = {
		'AnchorID': [5, 19, 13],
		'x': [8.3, 7.79, 0.92],
		'y': [4.085, 0.96, 0.16],
		'z': [1.945, 1.64, 1.21],
		'tdoa_of_anchors': [-1.129004, -5.36414]
	}
	
	# Selected anchors for 2D positioning
	sel_anch_dict_1 = {
		'AnchorID': [5, 3, 7],
		'x': [7.79, 8.3, 0.92],
		'y': [0.96, 4.085, 0.16],
		'z': [1.64, 1.945, 1.21],
		'tdoa_of_anchors': [-0.3559632, 1.094042]
	}

# ----------------------------

	# Selected anchors for 1D positioning
	sel_anch_dict_2 = {
		'AnchorID': [4, 5],
		'x': [2.88, 5.4],
		'y': [-1.69, 8.6],
		'z': [1.28, 1.28],
		'tdoa_of_anchors': [4.99]
	}

	# Selected anchors for 1D positioning
	sel_anch_dict_3 = {
		'AnchorID': [4, 5],
		'x': [2.88, 5.4],
		'y': [-1.69, 8.6],
		'z': [1.28, 1.28],
		'tdoa_of_anchors': [4.87]
	}

# ----------------------------

	# Selected anchors for 3D positioning
	sel_anch_dict_4 = {
		'AnchorID': [12, 8, 20, 23],
		'x': [1.84, 2.25, 4.265, 4.21],
		'y': [0.835, 3.15, 0.45, 2.6],
		'z': [2.84, 0.11, 0.11, 2.84],
		'tdoa_of_anchors': [0.71012035, -0.088364648, 1.2275097]
	}

	# Selected anchors for 3D positioning
	sel_anch_dict_5 = {
		'AnchorID': [13, 21, 16, 9],
		'x': [1.84, 4.265, 4.21, 2.25],
		'y': [0.835, 0.45, 2.6, 3.15],
		'z': [2.84, 0.11, 2.84, 0.11],
		'tdoa_of_anchors': [0.70016383, -1.1483716, 0.54085243]
	}

# ----------------------------
	"""
	def test_localization_mode(self):

		tested = pn.localization_mode()
		self.assertTrue(self.assertEqual(tested, 1) or self.assertEqual(tested, 2) or self.assertEqual(tested, 3))
	"""
	def test0_find_min_id_ind(self):	# 2D

		tested = find_min_id_ind(self.sel_anch_dict_0)
		test_result = int(0)

		self.assertEqual(tested, test_result)
	

	def test1_find_min_id_ind(self):	# 2D

		tested = pn.find_min_id_ind(self.sel_anch_dict_1)
		test_result = int(1)

		self.assertEqual(tested, test_result)


	def test2_find_min_id_ind(self):	# 1D

		tested = pn.find_min_id_ind(self.sel_anch_dict_2)
		test_result = int(0)

		self.assertEqual(tested, test_result)


	def test4_find_min_id_ind(self):	# 3D

		tested = pn.find_min_id_ind(self.sel_anch_dict_4)
		test_result = int(1)

		self.assertEqual(tested, test_result)


	def test5_find_min_id_ind(self):	# 3D

		tested = pn.find_min_id_ind(self.sel_anch_dict_5)
		test_result = int(3)

		self.assertEqual(tested, test_result)

# ----------------------------

	def test0_find_anchor_s(self):		# 2D

		min_id_ind = int(0)
		selected_x = list(self.sel_anch_dict_0['x'])
		selected_y = list(self.sel_anch_dict_0['y'])
		selected_z = list(self.sel_anch_dict_0['z'])

		tested = pn.find_anchor_s(min_id_ind, selected_x, selected_y, selected_z)
		test_result = list([8.3, 4.085, 1.945])

		self.assertListEqual(tested, test_result)

	def test1_find_anchor_s(self):		# 2D

		min_id_ind = int(1)
		selected_x = list(self.sel_anch_dict_1['x'])
		selected_y = list(self.sel_anch_dict_1['y'])
		selected_z = list(self.sel_anch_dict_1['z'])

		tested = pn.find_anchor_s(min_id_ind, selected_x, selected_y, selected_z)
		test_result = list([8.3, 4.085, 1.945])

		self.assertListEqual(tested, test_result)


	def test2_find_anchor_s(self):		# 1D

		min_id_ind = int(0)
		selected_x = list(self.sel_anch_dict_2['x'])
		selected_y = list(self.sel_anch_dict_2['y'])
		selected_z = list(self.sel_anch_dict_2['z'])

		tested = pn.find_anchor_s(min_id_ind, selected_x, selected_y, selected_z)
		test_result = list([2.88, -1.69, 1.28])

		self.assertListEqual(tested, test_result)


	def test4_find_anchor_s(self):		# 3D

		min_id_ind = int(1)
		selected_x = list(self.sel_anch_dict_4['x'])
		selected_y = list(self.sel_anch_dict_4['y'])
		selected_z = list(self.sel_anch_dict_4['z'])

		tested = pn.find_anchor_s(min_id_ind, selected_x, selected_y, selected_z)
		test_result = list([2.25, 3.15, 0.11])

		self.assertListEqual(tested, test_result)


	def test5_find_anchor_s(self):		# 3D

		min_id_ind = int(3)
		selected_x = list(self.sel_anch_dict_5['x'])
		selected_y = list(self.sel_anch_dict_5['y'])
		selected_z = list(self.sel_anch_dict_5['z'])

		tested = pn.find_anchor_s(min_id_ind, selected_x, selected_y, selected_z)
		test_result = list([2.25, 3.15, 0.11])

		self.assertListEqual(tested, test_result)

# ----------------------------

	def test0_find_anchor_a(self):		# 2D

		min_id_ind = int(0)
		selected_x = list(self.sel_anch_dict_0['x'])
		selected_y = list(self.sel_anch_dict_0['y'])
		selected_z = list(self.sel_anch_dict_0['z'])

		tested = pn.find_anchor_a(self.sel_anch_dict_0, min_id_ind, selected_x, selected_y, selected_z)
		test_result = [0.92, 0.16, 1.21], 2

		self.assertEqual(tested, test_result)


	def test1_find_anchor_a(self):		# 2D

		min_id_ind = int(0)
		selected_x = list(self.sel_anch_dict_1['x'])
		selected_y = list(self.sel_anch_dict_1['y'])
		selected_z = list(self.sel_anch_dict_1['z'])

		tested = pn.find_anchor_a(self.sel_anch_dict_1, min_id_ind, selected_x, selected_y, selected_z)
		test_result = [0.92, 0.16, 1.21], 2

		self.assertEqual(tested, test_result)

	
	def test2_find_anchor_a(self):		# 1D

		min_id_ind = int(0)
		selected_x = list(self.sel_anch_dict_2['x'])
		selected_y = list(self.sel_anch_dict_2['y'])
		selected_z = list(self.sel_anch_dict_2['z'])

		tested = pn.find_anchor_a(self.sel_anch_dict_2, min_id_ind, selected_x, selected_y, selected_z)
		test_result = [5.4, 8.6, 1.28], 1

		self.assertEqual(tested, test_result)


	def test4_find_anchor_a(self):		# 3D

		min_id_ind = int(1)
		selected_x = list(self.sel_anch_dict_4['x'])
		selected_y = list(self.sel_anch_dict_4['y'])
		selected_z = list(self.sel_anch_dict_4['z'])

		tested = pn.find_anchor_a(self.sel_anch_dict_4, min_id_ind, selected_x, selected_y, selected_z)
		test_result = [4.21, 2.6, 2.84], 3

		self.assertEqual(tested, test_result)


	def test5_find_anchor_a(self):		# 3D

		min_id_ind = int(3)
		selected_x = list(self.sel_anch_dict_5['x'])
		selected_y = list(self.sel_anch_dict_5['y'])
		selected_z = list(self.sel_anch_dict_5['z'])

		tested = pn.find_anchor_a(self.sel_anch_dict_5, min_id_ind, selected_x, selected_y, selected_z)
		test_result = [4.21, 2.6, 2.84], 2

		self.assertEqual(tested, test_result)

# ----------------------------

	def test0_find_anchor_b(self):		# 2D

		min_id_ind = int(0)
		anch_a_ind = int(2)
		selected_x = list(self.sel_anch_dict_0['x'])
		selected_y = list(self.sel_anch_dict_0['y'])
		selected_z = list(self.sel_anch_dict_0['z'])

		tested = pn.find_anchor_b(self.sel_anch_dict_0, min_id_ind, anch_a_ind, selected_x, selected_y, selected_z)
		test_result = [7.79, 0.96, 1.64], 1

		self.assertEqual(tested, test_result)


	def test1_find_anchor_b(self):		# 2D

		min_id_ind = int(1)
		anch_a_ind = int(2)
		selected_x = list(self.sel_anch_dict_1['x'])
		selected_y = list(self.sel_anch_dict_1['y'])
		selected_z = list(self.sel_anch_dict_1['z'])

		tested = pn.find_anchor_b(self.sel_anch_dict_1, min_id_ind, anch_a_ind, selected_x, selected_y, selected_z)
		test_result = [7.79, 0.96, 1.64], 0

		self.assertEqual(tested, test_result)


	def test4_find_anchor_b(self):		# 3D

		min_id_ind = int(1)
		anch_a_ind = int(3)
		selected_x = list(self.sel_anch_dict_4['x'])
		selected_y = list(self.sel_anch_dict_4['y'])
		selected_z = list(self.sel_anch_dict_4['z'])

		tested = pn.find_anchor_b(self.sel_anch_dict_4, min_id_ind, anch_a_ind, selected_x, selected_y, selected_z)
		test_result = [4.265, 0.45, 0.11], 2

		self.assertEqual(tested, test_result)


	def test5_find_anchor_b(self):		# 3D

		min_id_ind = int(3)
		anch_a_ind = int(2)
		selected_x = list(self.sel_anch_dict_5['x'])
		selected_y = list(self.sel_anch_dict_5['y'])
		selected_z = list(self.sel_anch_dict_5['z'])

		tested = pn.find_anchor_b(self.sel_anch_dict_5, min_id_ind, anch_a_ind, selected_x, selected_y, selected_z)
		test_result = [4.265, 0.45, 0.11], 1

		self.assertEqual(tested, test_result)

# ----------------------------

	def test4_find_anchor_c(self):		# 3D

		min_id_ind = int(1)
		anch_a_ind = int(3)
		anch_b_ind = int(2)
		selected_x = list(self.sel_anch_dict_4['x'])
		selected_y = list(self.sel_anch_dict_4['y'])
		selected_z = list(self.sel_anch_dict_4['z'])

		tested = pn.find_anchor_c(self.sel_anch_dict_4, min_id_ind, anch_a_ind, anch_b_ind, selected_x, selected_y, selected_z)
		test_result = [1.84, 0.835, 2.84]

		self.assertEqual(tested, test_result)
	
	def test5_find_anchor_c(self):		# 3D

		min_id_ind = int(3)
		anch_a_ind = int(2)
		anch_b_ind = int(1)
		selected_x = list(self.sel_anch_dict_5['x'])
		selected_y = list(self.sel_anch_dict_5['y'])
		selected_z = list(self.sel_anch_dict_5['z'])

		tested = pn.find_anchor_c(self.sel_anch_dict_5, min_id_ind, anch_a_ind, anch_b_ind, selected_x, selected_y, selected_z)
		test_result = [1.84, 0.835, 2.84]

		self.assertEqual(tested, test_result)

# ----------------------------

	def test0_calc_pos_2d_3a_ite(self):		# 2D
	
		anch_s = [8.3, 4.085, 1.945]
		anch_a = [0.92, 0.16, 1.21]
		anch_b = [7.79, 0.96, 1.64]
		dbs = -1.129004
		das = -5.364140
		tag_z = 1.72
		tested = pn.calc_pos_2d_3a_ite(anch_s, anch_a, anch_b, tag_z, das, dbs)
		test_result = [2.038855, 1.240696, 1.72]
		self.assertAlmostEqual(tested[0], test_result[0], 4)
		self.assertAlmostEqual(tested[1], test_result[1], 4)
		self.assertAlmostEqual(tested[2], test_result[2], 4)


	def test1_calc_pos_2d_3a_ite(self):		# 2D

		anch_s = [8.3, 4.085, 1.945]
		anch_a = [0.92, 0.16, 1.21]
		anch_b = [7.79, 0.96, 1.64]
		dbs = -0.3559632
		das = 1.094042
		tag_z = 1.72

		tested = pn.calc_pos_2d_3a_ite(anch_s, anch_a, anch_b, tag_z, das, dbs)
		test_result = [4.94928, 2.635953, 1.72]

		self.assertAlmostEqual(tested[0], test_result[0], 4)
		self.assertAlmostEqual(tested[1], test_result[1], 4)
		self.assertAlmostEqual(tested[2], test_result[2], 4)

	
	def test2_calc_dist_1d_2a_ite(self):	# 1D

		anch_s = [2.88, -1.69, 1.28]
		anch_a = [5.4, 8.6, 1.28]
		das = 4.99
		L1 = [3.36, 0.88, 0.84]
		L2 = [3.36, 1.76, 0.84]

		tested = pn.calc_dist_1d_2a_ite(anch_s, anch_a, L1, L2, das)
		test_result = [3.36, 1.0635, 0.84, 0.1835423]

		self.assertAlmostEqual(tested[0], test_result[0], 4)
		self.assertAlmostEqual(tested[1], test_result[1], 4)
		self.assertAlmostEqual(tested[2], test_result[2], 4)
		self.assertAlmostEqual(tested[3], test_result[3], 4)


	def test3_calc_dist_1d_2a_ite(self):	# 1D

		anch_s = [2.88, -1.69, 1.28]
		anch_a = [5.4, 8.6, 1.28]
		das = 4.87
		L1 = [3.36, 0.88, 0.84]
		L2 = [3.36, 1.76, 0.84]

		tested = pn.calc_dist_1d_2a_ite(anch_s, anch_a, L1, L2, das)
		test_result = [3.36, 1.1256026, 0.84, 0.2456027]

		self.assertAlmostEqual(tested[0], test_result[0], 4)
		self.assertAlmostEqual(tested[1], test_result[1], 4)
		self.assertAlmostEqual(tested[2], test_result[2], 4)
		self.assertAlmostEqual(tested[3], test_result[3], 4)
	

	def test4_calc_pos_3d_4a_ite(self):		# 3D

		anch_s = [2.25, 3.15, 0.11]
		anch_a = [4.21, 2.6, 2.84]
		anch_b = [4.265, 0.45, 0.11]
		anch_c = [1.84, 0.835, 2.84]
		das = 1.1398246
		dbs = -0.070313506
		dcs = 0.75415739

		tested = pn.calc_pos_3d_4a_ite(anch_s, anch_a, anch_b, anch_c, das, dbs, dcs)
		test_result = [2.794704, 1.370288, 0.4022198]

		self.assertAlmostEqual(tested[0], test_result[0], 4)
		self.assertAlmostEqual(tested[1], test_result[1], 4)
		self.assertAlmostEqual(tested[2], test_result[2], 4)


	def test5_calc_pos_3d_4a_ite(self):		# 3D

		anch_s = [2.25, 3.15, 0.11]
		anch_a = [4.21, 2.6, 2.84]
		anch_b = [4.265, 0.45, 0.11]
		anch_c = [1.84, 0.835, 2.84]
		das = 0.54085243
		dbs = -1.1483716
		dcs = 0.70016383

		tested = pn.calc_pos_3d_4a_ite(anch_s, anch_a, anch_b, anch_c, das, dbs, dcs)
		test_result = [3.574661, 1.303216, 0.4674477]

		self.assertAlmostEqual(tested[0], test_result[0], 4)
		self.assertAlmostEqual(tested[1], test_result[1], 4)
		self.assertAlmostEqual(tested[2], test_result[2], 4)


if __name__ == '__main__':
	import rosunit
	# rosunit.unitrun(PKG, NAME, TestPositioningNode, sysargs = "--cov", coverage_packages = ['indoor_localization', 'rospy', 'rosunit'])
	rosunit.unitrun(PKG, NAME, TestPositioningNode, sysargs = None, coverage_packages = [str(PKG)])
