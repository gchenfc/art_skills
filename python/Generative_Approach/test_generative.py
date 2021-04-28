"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit test for generative approach to strokes, using methodology described in:
Berio17eurographics_HandwritingTrajectories.pdf

Author: Juan-Diego Florez, Frank Dellaert, Sang-Won Leigh
"""
import unittest
from unittest.mock import patch
import numpy as np
#from gtsam.utils.test_case import GtsamTestCase


class GetParameters:
    """A class for recieving user input."""

    def get_num_pts(self):
        num_pts = int(input("Enter the number of points: "))
        #print(num_pts)
        return(num_pts)

    def get_points(self, num_pts):
        """Prompt the user for target points, p_i (i = [0:num_pts])"""
        for i in range(num_pts):
            #print("point", i, ":")
            pt_input = str(input("Enter a comma separated point 'x,y'"))
            pt_string = pt_input.split(',')
            pt_int = list(map(int, pt_string))
            if i == 0:
               T_pts = np.array(pt_int)
            else:
                T_pts_n = np.array(pt_int)
                T_pts = np.vstack((T_pts, T_pts_n))
        return (T_pts)

    def get_amplitudes(self, num_c):
        """Prompt the user for amplitude, D"""
        for i in range(num_c):
            D = int(input("Enter the amplitude: "))
            if i == 0:
                Ds = np.array(D)
            else:
                Ds_n = np.array(D)
                Ds = np.append(Ds, Ds_n)
        return(Ds)      

class GeneratedCurve:
    """Parameter processing for generative approach."""

    def find_theta(self, points):
        """find th_i from the norm and orientation of the vector p_{i}-p_{i-1}"""


class test_gen(unittest.TestCase):
    """Test generative approach to lognormal curves for art skills project."""
    num_of_pts = '3' # number of points being input, 2 points defines 1 curve
    string_of_pts_1 = '1,1' # first target point
    string_of_pts_2 = '5,5' # second target point
    string_of_pts_3 = '9,9'
    amplitude1 = '1' # amplitude of first curve
    amplitude2 = '2' # amplitude of second curve

    @patch('builtins.input', side_effect=[num_of_pts, string_of_pts_1, string_of_pts_2, string_of_pts_3, amplitude1, amplitude2])
    def test_get_param(self, mock_input):
        """Test the get input methods"""
        param = GetParameters() # call class
        pt_number = param.get_num_pts() # get number of points
        curve_num = pt_number - 1 # number of curves
        pt_array = param.get_points(pt_number) # get array of points
        print('points', pt_array)
        np.testing.assert_equal(pt_array, ([1,1],[5,5],[9,9])) # Test user-input point array (Success!)
        D_array = param.get_amplitudes(curve_num) # get amplitudes
        print('amplitudes', D_array)
        np.testing.assert_equal(D_array, [1,2]) # Test user-input amplitude array (Success!)       

    def test_generate(self):
        """Test the processing and plotting method."""

    # def testgen(self):
    #     """Prompt user for parameters, generate curve, plot"""
    #     skills = ArtSkills()
    #     captured_parameters = skills.capture_input()
    #     output_curve = skills.generate_curve()
    #     skills.plot_curve(output_curve)


if __name__ == "__main__":
    unittest.main()



# class ArtSkills:
#     """Top-level class."""

#     def capture_input(self):
#         """Asks for user-defined input parameters."""
#         captured_parameters = capture_points()

#     def generate_curve(self):
#         """ Infer a set of parameters that we think capture the artistic intent
#             of the artist.
#         """
#         # initialize parameters = PerceivedParameters()
#         # optimize for parameters such that we minimize:
#         #   diff(render_trajectory(parameters), captured_trajectory)
#         # typically done with GTSAM, by creating a facture graph

#     def plot_curve(self):
#         """Render plotly figure that reproduces artistic intent."""
#         #generated_stroke = self.generated_stroke(perceived)
#         # now render with plotly express