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
import matplotlib.pyplot as plt
from scipy import special
#from gtsam.utils.test_case import GtsamTestCase

class InputParameters:
    """A class for recieving user input."""

    def get_input(self, input_string):
        """Prompt user for input"""
        input_val = float(input(input_string))
        return(input_val)
    
    def step(self):
        "Prompt user for the sample rate/timestep of discrete points"
        t_step = float(input("Enter the timestep size: "))
        return(t_step)

    def num_pts(self):
        """Prompt the user for the amount of target points"""
        num_pts = int(input("Enter the number of points: "))
        return(num_pts)

    def num_curves(self, num_pts):
        """Find the number of curves defined by the target points"""
        num_curves = num_pts - 1 # number of curves
        return(num_curves)

    def points(self, num_pts):
        """Prompt the user for target points, p_i (i = [0:num_pts])"""
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
    
    def shape_param(self, num_c):
        """Prompt the user for the shape parameter (Ac) used in intermediate parameterization"""
        for i in range(num_c):
            Ac = float(input("Enter the shape parameter value: "))
            if i == 0:
                Acs = np.array(Ac)
            else:
                Acs_n = np.array(Ac)
                Acs = np.append(Acs, Acs_n)
        return(Acs)
        
    def period(self, num_c):
        """Prompt the user for the period of each curve"""
        for i in range(num_c):
            Ti = float(input("Enter the period: "))
            if i == 0:
                Tis = np.array(Ti)
            else:
                Tis_n = np.array(Ti)
                Tis = np.append(Tis, Tis_n)
        return(Tis)

class GeneratePoints:
    """Parameter processing for generative approach."""
    
    # Define mu and sigma as described below, above their functions. This comes from "Dijoua08EPM" paper
    # sigma, according to Berio17, is the response time in a logarithmic time scale
    def sigma(self, Ac):
        """Use the shape parameter to estimate the log response time, sigma"""
        sig = np.empty([len(Ac)])
        for i in range(len(Ac)):
            sig[i] = np.sqrt(-np.log(1 - Ac[i])) # assume ln, could be log10
        return sig

    # mu, according to Berio17, is the stroke delay in a logarithmic time scale
    def mu(self, sigs, Ts):
        """Use sigmas and period to estimate the stroke delay, mu"""
        u = np.empty([len(sigs)])
        for i in range(len(sigs)):
            u[i] = 3*sigs[i] - np.log((-1 + np.exp(6*sigs[i]))/Ts[i]) # again assuming ln
        return u

    def D_and_theta(self, num_c, points):
        """Find th_i from the norm and orientation of the vector p_{i}-p_{i-1}"""
        for i in range(num_c):
            v_array = points[i+1]-points[i]
            amplitude = norm(v_array,2)
            frac = v_array[1]/v_array[0] # delta y/ delta x
            theta_orient = np.arctan(frac) # angle
            if i == 0:
                dtheta = np.array([amplitude, theta_orient])
            else:
                dtheta_n = np.array([amplitude, theta_orient])
                dtheta = np.vstack((dtheta, dtheta_n))
        return(dtheta)

    def weight(self, tstep, T, dthetas, sigma, mu):
        """Calculate the weight of the stroke."""
        v_length = dthetas[:,0]
        w = []
        w_n = []
        for i in range(len(v_length)):
            steps = round(T[i]/tstep)
            for t in range(1,steps+1): # assign weight to each step
                time = t*tstep
                z = (np.log(time - 0) - mu[i])/(sigma[i]*np.sqrt(2)) # uses ln as assumed before, based on Berio eqtn
                erf_val = special.erf(z)
                w_val = 0.5*(1 + erf_val)
                w_n.append(w_val)
            # need an intermediate list to create list of lists of weights
            w_intermediate = w_n.copy()
            w.append(w_intermediate)
            w_n.clear()
        return(w)

    def displacement(self, tstep, T, dthetas, deltas, weights):
        """Generate stroke by weighted displacements at time (t)"""
        D_vec = dthetas[:,0]
        angle_vec = dthetas[:,1]
        d_list = []
        d = []
        for i in range(0,len(D_vec)):
            steps = round(T[i]/tstep)
            theta_0 = angle_vec[i] + (np.pi + deltas[i])/2
            for t in range(0,steps-1):
                if norm(deltas[i]) > (1*10**(-10)):
                    dx = D_vec[i]*((np.cos(theta_0 - deltas[i]*weights[i][t]) - np.cos(theta_0))*(2*np.sin(deltas[i]/2))**-1)
                    dy = D_vec[i]*((np.sin(theta_0 - deltas[i]*weights[i][t]) - np.sin(theta_0))*(2*np.sin(deltas[i]/2))**-1)
                else:
                    dx = D_vec[i]*(np.cos(angle_vec[i])*weights[t])
                    dy = D_vec[i]*(np.sin(angle_vec[i])*weights[t])
                if t == 0:
                    disp = np.array([dx, dy])
                else:
                    disp_n = np.array([dx, dy])
                    disp = np.vstack((disp, disp_n))
            d_list.append(disp)
        return(d_list)

    # this function currently only works for a single defined stroke. Second stroke points are calculated from first stroke origin
    def points(self, first_point, displacements):
        """Use displacements to get list of points."""
        for c in range(len(displacements)):
            if c == 0:
                px = first_point[c][0]
                py = first_point[c][1]
                fpoint = np.array([px, py])
                print('fpoint1', fpoint)
                print('len1', len(displacements[c]))
                point = fpoint
                point_arr = fpoint
                print('pa_0', point_arr)
            else:
                px = first_point[c][0]
                py = first_point[c][1]
                fpoint = np.array([px, py])
                print('fpoint2', fpoint)
                print('len2', len(displacements[c]))
                point = fpoint
            for i in range(len(displacements[c])):
                px = np.add(fpoint[0], displacements[c][i][0])
                py = np.add(fpoint[1], displacements[c][i][1])
                p = [px, py]
                point = np.vstack([point, p])
            point_arr = np.vstack([point_arr, point])
            print('points', np.shape(point_arr))
        return point_arr

class RenderCurve:
    """Rendering the final plot."""

    def plot_geometry(self, pts, des_pts):
        """Plot the points with matplotlib"""
        x1, y1 = pts.T
        plt.scatter(x1,y1)
        xd, yd = des_pts.T
        plt.scatter(xd, yd)
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.title('Generated Strokes')
        plt.show()

class ArtSkills:
    """Top-level class."""
    param = InputParameters() # call class

    def __init__(self, step_size, num_pts, pt, delta, Ac, T):
        self.step_size = step_size  # timestep size
        self.num_pts = num_pts      # number of target points, including origin
        self.pt = pt                # user-defined target point
        self.delta = delta          # central angle of the circular arc shape of the stroke
        self.Ac = Ac                # shape parameter that defines skewedness of the lognormal curve
        self.T = T                  # period of the stroke

    def capture_input(self):
        """Asks for user-defined input parameters."""
        timestep_str = "Enter the timestep size: "
        numpoints_str = "Enter the number of points: "
        parameter = np.array([param.get_input(timestep_str), param.get_input(numpoints_str)])

    def generate_curve(self):
        """ Infer a set of parameters that we think capture the artistic intent
            of the artist.
        """
        # initialize parameters = PerceivedParameters()
        # optimize for parameters such that we minimize:
        #   diff(render_trajectory(parameters), captured_trajectory)
        # typically done with GTSAM, by creating a facture graph

    def plot_curve(self):
        """Render plotly figure that reproduces artistic intent."""
        #generated_stroke = self.generated_stroke(perceived)
        # now render with plotly express  

class Test_All(unittest.TestCase): # I am not sure how to develop individual test cases, so I made a single one
    """Test generative approach to lognormal curves for art skills project."""

    """Simulated input values for testing"""
    time_step = '0.01' # "timestep" between points
    num_of_pts = '3' # number of target points being input, 2 points defines 1 curve
    string_of_pts_1 = '0,0' # first point, origin as user input
    string_of_pts_2 = '6,9' # second target point, as user input
    string_of_pts_3 = '10,4' # third target point, as user input TODO: get this to work with xi < xi-1. yi < yi-1 works iff xi > xi-1
    delta1 = '0.9944' # central angle of the circular arc shape of curve 1 (rad)
    delta2 = '0.3236' # central angle of the circular arc shape of curve 2 (rad)
    Ac1 = '0.1' # shape parameter that defines skewedness of the lognormal (Berio17euro...)(Plamondon2003)
    Ac2 = '0.1' # shape parameter that defines skewedness of the lognormal (Berio17euro...)(Plamondon2003)
    T1 = '0.7' # period of first curve
    T2 = '0.8' # period of second curve

    @patch('builtins.input', side_effect=[time_step, num_of_pts, string_of_pts_1, string_of_pts_2, string_of_pts_3, delta1, delta2, Ac1, Ac2, T1, T2])
    def test_input_sequence(self, mock_input):
        """Test the basic user input methods"""

        param = InputParameters() # call class
        timestep_str = "Enter the timestep size: "
        numpoints_str = "Enter the number of points: "
        point_str = "Enter point as 'x,y': "
        setup_param = np.array([param.get_input(timestep_str), param.get_input(numpoints_str)]) # get timestep and number of target points
        point_array = np.array([param.get_input(point_str), param.get_input(point_str)]) # get point string

        np.testing.assert_array_equal(setup_param, [0.01, 3])
        np.testing.assert_array_equal(point_array, [[0, 0], [6,9], [10,4]])
        np.testing.assert_array_equal(curve_param, [[0.9944, 0.3236], [0.1, 0.1], [0.7, 0.8]])
        
        """Test the point input sequence"""
        np.zeros(range(setup_param[1], 2)) # n-number of rows, 2 columns
        for i in range(setup_param[1]):
            param.get_input(timestep_str)


        np.testing.assert_array_equal(parameters, [])

        
        # param = InputParameters() # call class
        # step_size = param.step() # get timestep
        # np.testing.assert_equal(step_size, 0.01) # Test user-input method (Success!)
        
#         pt_number = param.num_pts() # get number of points
#         curve_num = param.num_curves(pt_number) # get number of curves
#         pt_array = param.points(pt_number) # get array of input points
#         deltas = param.internal_angle(curve_num) # get internal angle of arc
#         Ac_array = param.shape_param(curve_num) # get shape parameter
#         periods = param.period(curve_num) # get period of each stroke

#         """Test the generate methods"""
#         gen = GeneratePoints() # call class
#         D_theta = gen.D_and_theta(curve_num,pt_array) # find amplitude and direction of vector
#         sigmas = gen.sigma(Ac_array) 
#         mus = gen.mu(sigmas, periods)
#         weight_array = gen.weight(step_size, periods, D_theta, sigmas, mus) # need timestep and vector magnitude
#         valid_w = True # boolean to check that w exists exclusively within [0,1]
#         for l in range(0,len(weight_array)):
#             if any(t < 0 for t in weight_array[l]) or any(t > 1 for t in weight_array[l]):
#                 valid_w = False
#         self.assertTrue(valid_w) # Test weights are within [0, 1] (Success!)
#         displace_array = gen.displacement(step_size, periods, D_theta, deltas, weight_array)
#         print('disp', displace_array)
#         px1 = pt_array[0][0] + displace_array[0][-1][0]
#         py1 = pt_array[0][1] + displace_array[0][-1][1]
#         p_real1 = pt_array[1]
#         px2 = pt_array[1][0] + displace_array[1][-1][0]
#         py2 = pt_array[1][1] + displace_array[1][-1][1]
#         p_real2 = pt_array[2]
#         print('actual', [px2, py2], 'desired', pt_array[2])
#         np.testing.assert_array_almost_equal([px1, py1], p_real1, 1)
#         np.testing.assert_array_almost_equal([px2, py2], p_real2, 1)
#         point_array = gen.points(pt_array, displace_array)
#         #print(point_array)

        
#         """Test plotting"""
#         ren = RenderCurve() # call class
#         ren.plot_geometry(point_array, pt_array)

if __name__ == "__main__":
    unittest.main()