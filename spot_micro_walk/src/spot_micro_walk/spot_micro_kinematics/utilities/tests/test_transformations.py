"""Tests for the transformations module"""

from .. import transformations
import unittest
import math
import numpy as np

d2r = math.pi/180 # deg to rad conversion
r2d =  180/math.pi # rad to deg conversion

class TestRotationMatrices(unittest.TestCase):
    '''Tests rotation matrices'''

    def test_rotx(self):
        '''Test the x rotation matrix via a contrived example'''
        rot = transformations.rotx(90*d2r)
        known_true_rot = np.array(
            [   [1,   0,  0],
                [0,   0, -1],
                [0,   1,  0]    ])

        try:
            np.testing.assert_array_almost_equal(rot, known_true_rot)
            res = True
        except AssertionError as err:
            res = False
            print (err)
        self.assertTrue(res)

    def test_roty(self):
        '''Test the y rotation matrix via a contrived example'''
        rot = transformations.roty(90*d2r)
        known_true_rot = np.array(
            [   [0,   0,  1],
                [0,   1,  0],
                [-1,  0,  0]    ])

        try:
            np.testing.assert_array_almost_equal(rot, known_true_rot)
            res = True
        except AssertionError as err:
            res = False
            print (err)
        self.assertTrue(res)

    def test_rotz(self):
        '''Test the z rotation matrix via a contrived example'''
        rot = transformations.rotz(90*d2r)
        known_true_rot = np.array(
            [   [0,  -1,  0],
                [1,   0,  0],
                [0,   0,  1]    ])

        try:
            np.testing.assert_array_almost_equal(rot, known_true_rot)
            res = True
        except AssertionError as err:
            res = False
            print (err)
        self.assertTrue(res)

    def test_rotxyz(self):
        '''Test the x,y,z rotation matrix via a contrived example'''
        
        rot = transformations.rotxyz(90*d2r,90*d2r,90*d2r)

        known_true_rot = np.array(
            [   [0,   0,  1],
                [0,  -1,  0],
                [1,   0,  0]    ])

        try:
            np.testing.assert_array_almost_equal(rot, known_true_rot)
            res = True
        except AssertionError as err:
            res = False
            print (err)
        self.assertTrue(res)

    def test_translate(self):
        '''Test the linear translation on a vector example'''
        
        vec = np.array([1,2,3,1])

        tranlate_matrix = transformations.homog_transxyz(1,2,3)

        vec_translated = tranlate_matrix.dot(vec)

        known_answer = np.array([2,4,6,1])

        try:
            np.testing.assert_array_almost_equal(vec_translated, known_answer)
            res = True
        except AssertionError as err:
            res = False
            print (err)
        self.assertTrue(res)

    def test_homog_rotxyz(self):
        '''Test the homogeneous rotation matrix generation'''

        hr = transformations.homog_rotxyz(90*d2r,90*d2r,90*d2r)

        known_answer = np.array([[0,0,1,0],
                                 [0,-1,0,0],
                                 [1,0,0,0],
                                 [0,0,0,1]])

        try:
            np.testing.assert_array_almost_equal(hr, known_answer)
            res = True
        except AssertionError as err:
            res = False
            print (err)
        self.assertTrue(res)

    def test_homog_transform(self):
        '''Test the homogeneous transformation via a contrived example'''
        
        ht = transformations.homog_transform(90*d2r,90*d2r,90*d2r,1,2,3)

        known_true_rot = np.array(
            [   [0,   0,  1,  3],
                [0,  -1,  0,  -2],
                [1,   0,  0,  1],
                [0,   0,  0,  1]    ])

        try:
            np.testing.assert_array_almost_equal(ht, known_true_rot)
            res = True
        except AssertionError as err:
            res = False
            print (err)
        self.assertTrue(res)
        
    def test_homog_transform_inverse(self):
        '''Test a homogeneous transformation inverse'''

        # Test by running a forward transformation on a set of coordinates, then reversing 
        # it via the inverse, and making sure the two coordinates match

        phi = 10*d2r
        theta = 20*d2r
        psi = -30*d2r

        x_t = -5
        y_t = 10
        z_t = 2.56

        test_x = 0.23
        test_y = 100
        test_z = -62
        orig_coords = np.array([test_x,test_y,test_z,1])

        # Get homogeneous transformation

        ht = transformations.homog_transform(phi,theta,psi,x_t,y_t,z_t)
        # Forward transformation
        transformed_coords = ht.dot(orig_coords)

        # Inverse of homogeneous transform
        inv_ht = transformations.ht_inverse(ht)

        # Getting back original coordinates
        test_coords = inv_ht.dot(transformed_coords) 

        try:
            np.testing.assert_array_almost_equal(orig_coords, test_coords)
            res = True
        except AssertionError as err:
            res = False
            print (err)
        self.assertTrue(res)


if __name__ == '__main__':
    unittest.main()
