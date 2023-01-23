"""
GTSAM Copyright 2010-2022, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit test loading letter data
Author: Gerry Chen
"""

import unittest
import numpy as np
from gtsam.utils.test_case import GtsamTestCase
from gtsam.symbol_shorthand import X, P
from sln_stroke_fit import OptimizationLoggingParams
import sln_letter_fit
import loader


class TestLoader(GtsamTestCase):
    """Test loader.py."""

    def test_load_segments(self):
        """Test loading segments."""
        # Test loading strokes from first trajectory
        strokes = loader.load_segments('A', 1)
        self.assertEqual(len(strokes), 9, 'first trajectory in letter A should have 9 strokes')

        # Test loading all trajectories
        trajectories = loader.load_segments('A', None)
        self.assertEqual([len(trajectory) for trajectory in trajectories], [9, 1],
                         'loading letter A trajectories failed.')

if __name__ == "__main__":
    unittest.main()
