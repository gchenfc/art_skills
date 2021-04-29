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
from numpy.linalg import norm
#from gtsam.utils.test_case import GtsamTestCase


class GetParameters:
    """A class for recieving user input."""

    def get_step(self):
        "Prompt user for the sample rate/timestep of discrete points"
        t_step = float(input("Enter the number of points: "))
        return(t_step)

    def get_num_pts(self):
        """Prompt the user for the amount of target points"""
        num_pts = int(input("Enter the number of points: "))
        #print(num_pts)
        return(num_pts)

    def get_num_curves(self, num_pts):
        """Find the number of curves defined by the target points"""
        num_curves = num_pts - 1 # number of curves
        return(num_curves)

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

class GenerateCurve:
    """Parameter processing for generative approach."""

    def determine_weight(self, tstep):
        """Calculate the weight of the stroke."""

    def find_theta(self, num_c, points):
        """Find th_i from the norm and orientation of the vector p_{i}-p_{i-1}"""
        for i in range(num_c):
            v_array = points[i+1]-points[i]
            theta_norm = norm(v_array,2)
            #print('norm', theta_norm)
            frac = v_array[1]/v_array[0] # delta y/ delta x
            theta_orient = np.arctan(frac) # angle
            if i == 0:
                theta = np.array([theta_norm, theta_orient])
            else:
                theta_n = np.array([theta_norm, theta_orient])
                theta = np.vstack((theta, theta_n))
        print('theta', theta)
        return(theta)


class test_gen(unittest.TestCase): # I am not sure how to develop individual test cases, so I made a single one
    """Test generative approach to lognormal curves for art skills project."""
    step_size = '0.1' # "timestep" between points
    num_of_pts = '3' # number of points being input, 2 points defines 1 curve
    string_of_pts_1 = '1,1' # first target point, as user input
    string_of_pts_2 = '5,3' # second target point, as user input
    string_of_pts_3 = '9,9' # third target point, as user input
    amplitude1 = '1' # amplitude of first curve
    amplitude2 = '2' # amplitude of second curve

    @patch('builtins.input', side_effect=[step_size, num_of_pts, string_of_pts_1, string_of_pts_2, string_of_pts_3, amplitude1, amplitude2])
    def test_gen_approach(self, mock_input):
        """Test the get input methods"""
        param = GetParameters() # call class
        step_size = param.get_step() # get timestep
        pt_number = param.get_num_pts() # get number of points
        curve_num = param.get_num_curves(pt_number)
        pt_array = param.get_points(pt_number) # get array of points
        print('points', pt_array)
        np.testing.assert_equal(pt_array, ([1,1],[5,3],[9,9])) # Test user-input point array (Success!)
        D_array = param.get_amplitudes(curve_num) # get amplitudes
        print('amplitudes', D_array)
        np.testing.assert_equal(D_array, [1,2]) # Test user-input amplitude array (Success!)

        """Test the generate methods"""
        gen = GenerateCurve() # call class
        theta_array = gen.find_theta(curve_num,pt_array)
        # Test theta generate method (unsure what to put here)
        weight_array = gen.determine_weight(step_size)




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