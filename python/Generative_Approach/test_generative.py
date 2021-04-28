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


class CapturedParameters:
    """A class for recieving user input."""

    def capture_points(self):
        """Prompt the user for each parameter"""
        T_pts = np.empty([1, 2])
        num_pts = int(input("Enter the number of points: "))
        for i in range(num_pts):
            pt_input = str(input("Enter a comma separated point 'x,y'"))
            pt_string = pt_input.split(',')
            pt_int = list(map(int, pt_string))
            if i == 0:
               T_pts = np.array(pt_int)
            else:
                T_pts_n = np.array(pt_int)
                T_pts = np.vstack((T_pts, T_pts_n))
        return (T_pts)


class test_gen(unittest.TestCase):
    """Test generative approach to lognormal curves for art skills project."""
    num_of_pts = '2' # number of points being input
    string_of_pts_1 = '1,1' # first target point
    string_of_pts_2 = '5,5' # second target point

    @patch('builtins.input', side_effect=[num_of_pts, string_of_pts_1, string_of_pts_2])
    def test_capture(self, mock_input):
        """Test the capture input approach for target points"""
        cap = CapturedParameters()
        result = cap.capture_points()
        check1 = result[0,0]
        np.testing.assert_equal(result, ([1,1],[5,5])) # Success! We can now capture points from user input
        
        """Test the capture input approach for target points"""


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



# class GeneratedCurve:
#     """Parameter processing for generative approach."""

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