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

class ArtSkills:
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

    def test_perceived_parameters(self):
        """Test PerceivedParameters constructions."""
        target_points = np.array([[], [], []])
        segment_parameters = [
            {"sigma": 1.0},
            {"sigma": 2.0},
        ]
        parameters = PerceivedParameters(target_points, segment_parameters)
        self.assertEqual(parameters.num_segments(), 2)

    def test_render_trajectory(self):
        """Test render_trajectory method."""

    def test_perceive(self):
        """Test perceive method."""

    def test_whole_enchilada(self):
        """Capture, perceive, and render."""
        skills = ArtSkills()
        trajectory_filename = ""
        captured_trajectory = skills.read_2D_trajectory(trajectory_filename)
        perceived = skills.perceive(captured_trajectory)
        skills.render_with_plotly(perceived)


if __name__ == "__main__":
    unittest.main()
