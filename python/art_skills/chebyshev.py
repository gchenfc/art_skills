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
from matplotlib import pyplot as plt
from numpy.core.function_base import linspace
import gtsam
from gtsam.utils.test_case import GtsamTestCase


class alphabet_skills:
    """Top-level class."""

    def chebyshev_fit(self, data, time, p_order):
        """
        Fits a Chebyshev Basis and interpolates for x against t using GTSAM

        Args:
            data ([array]): [the x or y coordinates of a stroke]
            time ([array]): [the time for each x or y position]
            porder ([int]): [the order of chebyshev polynomial]
        """
        noise = gtsam.noiseModel.Unit.Create(1)
        self.x = [0., 0.5, 0.75]
        fit = gtsam.FitBasisChebyshev1Basis(data, noise, p_order)
        coeff = fit.parameters()
        basis = gtsam.Chebyshev1Basis
        interp = basis.WeightMatrix(p_order, time) @ coeff
        return(interp)

    def render_trajectory(self, traj, order):
        """
        Render trajectory that reproduces artistic intent.

        Args:
            traj ([array]): [the letters and strokes of each letter]
            order ([int]): [the order of chebyshev polynomial]
        """
        interp_x = []
        interp_y = []
        count_s = 0
        count_l = 0
        for letter in traj:
            for stroke in letter:
                traj_x = (stroke[:, 0] + count_l*0.7)
                traj_y = (stroke[:, 1])
                traj_t = linspace(0, 1, len(traj_x))
                zipx = zip(traj_t, traj_x)
                zipy = zip(traj_t, traj_y)
                data_x = dict(zipx)
                data_y = dict(zipy)
                interp_x.append(self.chebyshev_fit(data_x, traj_t, order))
                interp_y.append(self.chebyshev_fit(data_y, traj_t, order))
                count_s += 1
            count_l += 1
        return(interp_x, interp_y)


class TestWholeEnchilada(GtsamTestCase):
    """Test end-to-end for Chebyshev fitting."""

    def test_capture_trajectory(self):
        """Generate a trajectory."""
        skills = alphabet_skills()
        trajectory = skills.capture_trajectory(["A"])
        plt.plot(trajectory[0][0][:, 0], trajectory[0][0][:, 1])
        plt.plot(trajectory[0][1][:, 0], trajectory[0][1][:, 1])
        plt.show()

    def test_chebyshev(self):
        """Test Chebyshev fitting"""
        skills = alphabet_skills()
        x = np.array([0, 0.1, 1.5, 0.25, 0])
        t = np.array([0, 0.5,   1,  1.5, 2])
        order = 3
        zipx = zip(t, x)
        datax = dict(zipx)
        trajectory = skills.Chebyshev_fit(datax, t, order)
        plt.plot(t, x)
        plt.plot(t, trajectory)
        plt.show()

    def test_write_alphabet(self):
        """Capture/take input, perceive/process, and render the entire
        alphabet."""
        skills = alphabet_skills()
        capture = capture_trajectory()
        traj = capture.capture_trajectory(["Test"])
        xcoord, ycoord = skills.render_trajectory(traj, 12)
        for stroke in range(len(xcoord)):
            plt.plot(xcoord[stroke], ycoord[stroke])
        plt.axis("equal")
        plt.show()


if __name__ == "__main__":
    unittest.main()
