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

class InputParameters:
    """A class for recieving user input."""

    def step(self):
        "Prompt user for the sample rate/timestep of discrete points"
        t_step = float(input("Enter the number of points: "))
        return(t_step)

    def num_pts(self):
        """Prompt the user for the amount of target points"""
        num_pts = int(input("Enter the number of points: "))
        #print(num_pts)
        return(num_pts)

    def num_curves(self, num_pts):
        """Find the number of curves defined by the target points"""
        num_curves = num_pts - 1 # number of curves
        return(num_curves)

    def points(self, num_pts):
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

    def amplitudes(self, num_c):
        """Prompt the user for amplitude, D"""
        for i in range(num_c):
            D = int(input("Enter the amplitude: "))
            if i == 0:
                Ds = np.array(D)
            else:
                Ds_n = np.array(D)
                Ds = np.append(Ds, Ds_n)
        return(Ds)

    def internal_angle(self, num_c):
        """Prompt the user for the internal angle (delta)"""
        for i in range(num_c):
            delta = float(input("Enter the internal angle (rad): "))
            if i == 0:
                deltas = np.array(delta)
            else:
                deltas_n = np.array(delta)
                deltas = np.append(deltas, deltas_n)
        return(deltas)

class GenerateCurve:
    """Parameter processing for generative approach."""
    
    def sigma(self, thetas):
        """Use the "range rule to generate a rough approximate of std dev (TODO: find other means of finding sigma)"""
        v_length = thetas[:,0]
        sig = np.empty([len(v_length)])
        for i in range(0,len(v_length)):
            sig[i] = (v_length[i] - 0)/4
        return sig

    def mu(self, thetas):
        """Assumes equally distributed points from min to max (TODO: find other means of finding mu)"""
        v_length = thetas[:,0]
        u = np.empty([len(v_length)])
        for i in range(0,len(v_length)):
            u[i] = (v_length[i]/2)
        return u

    def erf(self, zt, z0):
        """Error function encountered in integrating the normal distribution (normalized form of Gaussian function)"""
        e = 2/(np.sqrt(np.pi)) * (np.exp(-zt**2) - np.exp(-z0**2))
        #print('error', e)
        return e

    def theta(self, num_c, points):
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
        #print('theta', theta)
        return(theta)

    def weight(self, tstep, thetas, sigma, mu):
        """Calculate the weight of the stroke."""
        v_length = thetas[:,0]
        w = []
        w_n = []
        for i in range(0,len(v_length)):
            steps = int(np.floor(v_length[i]/tstep))
            #print('steps', steps)
            for t in range(1,steps): # this is currently assigning a weight through each time step, but it may be that this should only be from tf - t0
                z0 = (np.log10(1 - 0) - mu[i])/(sigma[i]*np.sqrt(2)) # assumes t0 at 1 (can't be 0 or divide by 0 error) (TODO: change this to be t0, relative to the start of the curves) 
                zt = (np.log10(t - 0) - mu[i])/(sigma[i]*np.sqrt(2)) # uses log10 instead of ln, based on Berio eqtn
                #print('zt', zt)
                #print('t',t)
                #print('i', i)
                erf_val = self.erf(zt, z0)
                w_val = 0.5*(1 + erf_val)
                w_n.append(w_val)
            # need an intermediate list to create list of lists of weights
            w_intermediate = w_n.copy()
            w.append(w_intermediate)
            w_n.clear()
        #print('w', len(w))
        #print(len(w[1]))
        return(w)

    def displacement(self, tstep, thetas, amplitudes, deltas, weights):
        """Generate stroke by weighted displacements at time (t)"""


class test_gen(unittest.TestCase): # I am not sure how to develop individual test cases, so I made a single one
    """Test generative approach to lognormal curves for art skills project."""
    step_size = '0.1' # "timestep" between points
    num_of_pts = '3' # number of points being input, 2 points defines 1 curve
    string_of_pts_1 = '1,1' # first target point, as user input
    string_of_pts_2 = '5,3' # second target point, as user input
    string_of_pts_3 = '9,9' # third target point, as user input
    amplitude1 = '1' # amplitude of first curve
    amplitude2 = '2' # amplitude of second curve
    delta1 = '0.10472' # internal angle of the circular arc shape of curve 1 (6 deg)
    delta2 = '0.15708' # internal angle of the circular arc shape of curve 2 (9 deg)

    @patch('builtins.input', side_effect=[step_size, num_of_pts, string_of_pts_1, string_of_pts_2, string_of_pts_3, amplitude1, amplitude2, delta1, delta2])
    def test_gen_approach(self, mock_input):
        """Test the get input methods"""
        param = InputParameters() # call class
        step_size = param.step() # get timestep
        pt_number = param.num_pts() # get number of points
        curve_num = param.num_curves(pt_number)
        pt_array = param.points(pt_number) # get array of points
        #print('points', pt_array)
        np.testing.assert_equal(pt_array, ([1,1],[5,3],[9,9])) # Test user-input point array (Success!)
        D_array = param.amplitudes(curve_num) # get amplitudes
        #print('amplitudes', D_array)
        np.testing.assert_equal(D_array, [1,2]) # Test user-input amplitude array (Success!)
        deltas = param.internal_angle(curve_num)
        np.testing.assert_equal(deltas, [0.10472,0.15708]) # Test user-input amplitude array (Success!)

        """Test the generate methods"""
        gen = GenerateCurve() # call class
        theta_array = gen.theta(curve_num,pt_array) # find magnitude and angle of vector
        # Test theta generate method (unsure what to put here)
        sigmas = gen.sigma(theta_array)
        mus = gen.mu(theta_array)
        # print('sigma', sigmas)
        # print('mu', mus)
        weight_array = gen.weight(step_size, theta_array, sigmas, mus) # need timestep and vector magnitude
        # print(weight_array)
        valid_w = True # boolean to check that w exists exclusively within [0,1]
        for l in range(0,len(weight_array)):
            if any(t < 0 for t in weight_array[l]) or any(t > 1 for t in weight_array[l]):
                valid_w = False
        self.assertTrue(valid_w) # Test weights are within [0, 1] (Success!)


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