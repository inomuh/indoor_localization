#!/usr/bin/env python

import unittest
import math
from indoor_localization.localization_node import CalcPos2D3AItle

PKG = 'indoor_localization'
NAME = 'localization_node_test'

class TestLocalizationNode(unittest.TestCase):
	
	def test_localization_node(self):
		
		tagpos = CalcPos2D3AItle(8.3, 4.085, 1.945, 7.79, 0.96, 1.64, 0.92, 0.16, 1.21, 1.72, -1.053, -4.998)
		self.assertEquals(tagpos.TX_CM, 213)
		self.assertEquals(tagpos.TY_CM, 141)
		self.assertEquals(tagpos.TZ_CM, 172)
		
if __name__ == '__main__':
	import rosunit
	rosunit.unitrun(PKG, NAME, TestLocalizationNode, sysargs = None, coverage_packages=['indoor_localization'])
