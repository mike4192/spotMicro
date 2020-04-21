'''Tests the spot micro stick figure and spot micro leg classes'''

import unittest
from ..spot_micro_stick_figure import SpotMicroLeg
from ..utilities import spot_micro_kinematics as smk
from ..utilities import transformations
import numpy as np
from math import cos, sin, pi

d2r = pi/180
r2d = 180/pi

class TestSpotMicroLeg(unittest.TestCase):
    '''Tests spot micro leg class'''

    def test_leg(self):
        '''Test a spot micro leg with contrived angles and leg link lengths'''
        
        q1 = 0*d2r
        q2 = 0*d2r
        q3 = 30*d2r
        l1 = 1
        l2 = 2
        l3 = 3
        
        # Body coordinate system homogeneous transformation. Center at 0, no rotations
        body_ht = transformations.homog_transform(0,0,0,0,0,0)

        # Right front leg homogeneous transform. For test, create with 0 width and length, so
        # leg start position should be at origin aligned with global coordinate frame
        ht_rf = smk.t_rightfront(body_ht,0,0)

        leg = SpotMicroLeg(q1,q2,q3,l1,l2,l3,ht_rf,leg12=True)

        (p1,p2,p3,p4) = leg.get_leg_points()

        known_true_p1 = np.array([0,0,0])
        known_true_p2 = np.array([0,0,1])
        known_true_p3 = np.array([0,-2,1])
        known_true_p4 = np.array([l3*sin(q3),-l3*cos(q3)+-2,1])

        try:
            np.testing.assert_array_almost_equal(p1, known_true_p1)
            np.testing.assert_array_almost_equal(p2, known_true_p2)
            np.testing.assert_array_almost_equal(p3, known_true_p3)
            np.testing.assert_array_almost_equal(p4, known_true_p4)
            res = True
        except AssertionError as err:
            res = False
            print (err)
        self.assertTrue(res)
        