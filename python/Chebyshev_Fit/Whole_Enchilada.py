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
from gtsam.utils.test_case import GtsamTestCase


class CapturedTrajectory:
    """A class for the captured trajectory."""


class PerceivedParameters:
    """Parameters that we think capture the artistic intent."""
    # target points w, np.ndarray
    # list of segment parameters

    def __init__(self, target_points: np.ndarray, segment_parameters: dict):
        """Create from target points w and segment parameters.
          Parameters:
            target_points:
            segment_parameters: 
        """

    def num_segments(self):
        """Return the number of segments the user had in mind."""

class alphabet_skills:
    """Top-level class."""

    def read_2D_trajectory(self, trajectory_filename: str):
        """Read a 2D trajectory of user input from file."""

    def render_trajectory(self, perceived: PerceivedParameters):
        """Render trajectory that reproduces artistic intent."""

    def perceive(self, captured_trajectory: CapturedTrajectory):
        """ Infer a set of parameters that we think capture the artistic intent
            of the artist.
        """
        # initialize parameters = PerceivedParameters()
        # optimize for parameters such that we minimize:
        #   diff(render_trajectory(parameters), captured_trajectory)
        # typically done with GTSAM, by creating a facture graph

    def render_with_plotly(self, perceived: PerceivedParameters):
        """Render plotly figure that reproduces artistic intent."""
        new_trajectory = self.render_trajectory(perceived)
        # now render with plotly express


class TestWholeEnchilada(GtsamTestCase):
    """Test end-to-end for art skills project."""
    
    def test_capture(self):
        """Test the input capture method"""
        
    def test_render_trajectory(self):
        """Test render_trajectory method."""

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


