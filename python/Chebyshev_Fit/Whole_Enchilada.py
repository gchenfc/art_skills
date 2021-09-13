"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for the entire project, end-end
Author: Frank Dellaert, Sang-Won Leigh, JD Florez-Castillo
"""

import unittest

import numpy as np
from numpy import load
from matplotlib import pyplot as plt
from numpy.core.function_base import linspace
import gtsam
from gtsam.utils.test_case import GtsamTestCase


class Chebyshev_fit:
    """A class for processing/fitting the trajectory."""


class render_trajectory:
    """A class for processing/fitting the trajectory."""


class alphabet_skills:
    """Top-level class."""

    def create_V(self, input_sequence, step):
        """Define 2D trajectory from user input.

        Args:
            input_sequence ([list of strings]): [contains a sequence of
            letters of the alphabet]
            step ([int]): [description]

        Returns:
            trajectory ([list of lists of arrays])
        """
        letter = []
        trajectory = []
        width = 1.0  # width of the letters
        height = 1.0  # height of the letters
        m = np.hypot(width, height)  # slope of diagonal strokes

        for element in input_sequence:
            if element == "V":
                x1 = np.arange(0.0, width, step)
                y1 = np.empty(len(x1),)
                for i in range(len(x1)):
                    if x1[i] <= width/2:
                        y1[i] = -m*x1[i] + height
                    else:
                        y1[i] = m*x1[i] - height*2
                stroke1 = np.array([x1, y1])
                letter.append(stroke1)
                trajectory.append(letter)
            else:
                print("Invalid desired trajectory")
        return(trajectory[0])

    def capture_trajectory(self, input_sequence):
        """Read from mocap data

        Args:
            alphabet ([.npz]): [a bunch of .npy files]
        """
        data = np.load('all_letters.npz', allow_pickle=True)

        # data_a is a list of strokes, where each stroke is an Nx2 numpy array
        data_a = data['A']
        # for stroke in data_a:
        #     plt.plot(stroke[:, 0], stroke[:, 1])
        # plt.show()

        stroke1 = data_a[1]
        stroke2 = data_a[3]
        traj = np.array([stroke1, stroke2])
        plt.plot(stroke1[:, 0], stroke1[:, 1])
        plt.plot(stroke2[:, 0], stroke2[:, 1])
        plt.show()
        data_b = data['B']
        # for stroke in data_b:
        #     plt.plot(stroke[:, 0], stroke[:, 1])
        # plt.show()
        data_c = data['C']
        # for stroke in data_c:
        #     plt.plot(stroke[:, 0], stroke[:, 1])
        # plt.show()
        data_d = data['D']
        # for stroke in data_d:
        #     plt.plot(stroke[:, 0], stroke[:, 1])
        # plt.show()
        data_e = data['E']
        # for stroke in data_e:
        #     plt.plot(stroke[:, 0], stroke[:, 1])
        # plt.show()
        data_f = data['F']
        # for stroke in data_f:
        #     plt.plot(stroke[:, 0], stroke[:, 1])
        # plt.show()
        # trajectory = np.empty(len(input_sequence))
        # index = 0
        # print(data_a.shape)
        # for element in input_sequence:
        #     if element == "A":
        #         trajectory[index] = data_a
        #     elif element == "B":
        #         trajectory[index] = data_b
        #     elif element == "C":
        #         trajectory[index] = data_c
        #     elif element == "D":
        #         trajectory[index] = data_d
        #     elif element == "E":
        #         trajectory[index] = data_e
        #     elif element == "F":
        #         trajectory[index] = data_f
        #     else:
        #         print("Invalid desired trajectory")
        #     index += 1
        return(traj)

    def Chebyshev_fit(self, ):  # , read_2D: capture_trajectory):
        """Process the input and fit to Chebyshev using GTSAM"""
        # Call the GTSAM functions here

    def test_fit_basis_chebyshev1basis(self, data, time, p_order):
        """Fits a Chebyshev Basis and interpolates for x or y against t

        Args:
            data ([array]): [the x or y coordinates of a stroke]
        """
        # time = data[0]
        # sample = data[1]
        noise = gtsam.noiseModel.Unit.Create(1)
        self.x = [0., 0.5, 0.75]
        fit = gtsam.FitBasisChebyshev1Basis(data, noise, p_order)
        coeff = fit.parameters()
        print("COEFF", coeff)
        basis = gtsam.Chebyshev1Basis
        interp = basis.WeightMatrix(p_order, time) @ coeff
        return(interp)


    def render_trajectory(self):  # , Cheby: Chebyshev_fit):
        """Render trajectory that reproduces artistic intent."""
        return("""List of list of polynomials""")

    def generate_figure(self):  # , Cheby: Chebyshev_fit, render: render_trajectory):
        """Create results figure to compare trajectory with fit."""
        # Jupyter Notebook will call on this function for plotting.


class TestWholeEnchilada(GtsamTestCase):
    """Test end-to-end for art skills project."""

    # def test_V_trajectory(self):
    #     """Generate a trajectory."""
    #     skills = alphabet_skills()
    #     letters = ["V"]
    #     stepsize = 0.02
    #     trajectory = skills.create_V(letters, stepsize)
    #     x = trajectory[0][0]
    #     y = trajectory[0][1]
    #     plt.plot(x, y)
    #     plt.show()

    def test_capture_trajectory(self):
        """Generate a trajectory."""
        skills = alphabet_skills()
        trajectory = skills.capture_trajectory(["A"])
        return(trajectory)

    def test_chebyshev(self):
        skills = alphabet_skills()
        
        # traj_x = []
        # traj_y = []
        # traj_t = []
        # data_x = []
        # data_y = []
        interp_x = []
        interp_y = []
        count = 0
        order = 12
        traj = self.test_capture_trajectory()
        for stroke in traj:
            traj_x = (stroke[:, 0])
            traj_y = (stroke[:, 1])
            traj_t = linspace(0, 1, len(traj_x))
            zipx = zip(traj_t, traj_x)
            zipy = zip(traj_t, traj_y)
            data_x = dict(zipx)
            data_y = dict(zipy)
            interp_x.append(skills.test_fit_basis_chebyshev1basis(data_x, traj_t, order))
            interp_y.append(skills.test_fit_basis_chebyshev1basis(data_y, traj_t, order))
            plt.plot(interp_x[count], interp_y[count])
            count += 1
        plt.show()

    # def test_write_stroke(self):
    #     """Write a single stroke."""
    #     return ("""One polynomial""")

    # def test_write_letter(self):
    #     """Write a single letter."""
    #     return ("""list of polynomials describing one letter""")

    # def test_write_letters(self):
    #     """Write out a sequence of letters."""
    #     letter = self.test_write_letter
    #     return("""list of lists of polynomials""")

    # def test_write_alphabet(self):
    #     """Capture/take input, perceive/process, and render the entire
    #     alphabet."""
    #     skills = alphabet_skills()
    #     captured_trajectory = skills.capture_trajectory()
    #     cheby_out = skills.Chebyshev_fit(captured_trajectory)
    #     skills.render_trajectory(cheby_out)


if __name__ == "__main__":
    unittest.main()
