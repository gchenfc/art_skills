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

import gtsam
from gtsam.utils.test_case import GtsamTestCase
from gtsam.symbol_shorthand import B


class Chebyshev_fit:
    """A class for processing/fitting the trajectory."""
    
class render_trajectory:
    """A class for processing/fitting the trajectory."""

class alphabet_skills:
    """Top-level class."""

    def read_2D_trajectory(self, trajectory_filename: str):
        """Read a 2D trajectory of user input."""

    def Chebyshev_fit(self, read_2D: read_2D_trajectory):
        """Process the input and fit to Chebyshev using GTSAM"""

    def render_trajectory(self, Cheby: Chebyshev_fit):
        """Render trajectory that reproduces artistic intent."""

    def generate_figure(self, Cheby: Chebyshev_fit, render: render_trajectory):
        """Create results figure to compare trajectory with fit."""


class TestWholeEnchilada(GtsamTestCase):
    """Test end-to-end for art skills project."""
    
     # TODO 1: replace with actual import from python/gtsam/tests/test_basis.py
    def setUp(self): # replace with actual import from python/gtsam/tests/test_basis.py
        self.N = 2
        self.x = [0., 0.5, 0.75]
        self.interpx = np.linspace(0., 1., 10)
        self.noise = gtsam.noiseModel.Unit.Create(1)

    def evaluate(self, basis, fitparams, x): # TODO 1
        self.N = 2
        """
        Until wrapper for Basis functors are ready,
        this is how to evaluate a basis function.
        """
        return basis.WeightMatrix(self.N, x) @ fitparams

    def fit_basis_helper(self, fitter, basis, f=lambda x: x): # TODO 1
        """Helper method to fit data to a specified fitter using a specified basis."""
        data = {x: f(x) for x in self.x}
        fit = fitter(data, self.noise, self.N)
        coeff = fit.parameters()
        interpy = self.evaluate(basis, coeff, self.interpx)
        return interpy

    def test_fit_basis_chebyshev1basis(self):
        """Fit a Chebyshev1 basis."""

        f = lambda x: x
        interpy = self.fit_basis_helper(gtsam.FitBasisChebyshev1Basis,
                                        gtsam.Chebyshev1Basis, f)
        # test a basis by checking that the fit result matches the function at x-values interpx.
        np.testing.assert_almost_equal(interpy,
                                       np.array([f(x) for x in self.interpx]),
                                       decimal=7)

    def test_write_letter(self):
        """Fit a single trajectory."""
        return ("""One polynomial""")
    
    def test_write_letter(self):
        """Write a single letter."""
        return ("""list of polynomials describing one letter""")

    def test_write_letters(self):
        """Write out a sequence of letters."""
        letter = self.test_write_letter
        return("""list of lists of polynomials""")
    
    def test_write_alphabet(self):
        """Capture/take input, perceive/process, and render the entire alphabet."""
        skills = alphabet_skills()
        captured_trajectory = skills.capture_trajectory()
        cheby_out = skills.perceive(captured_trajectory)
        skills.render(cheby_out)


if __name__ == "__main__":
    unittest.main()


