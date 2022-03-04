"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for the entire project, end-end
Author: Frank Dellaert, Sang-Won Leigh, JD Florez-Castillo
"""

import unittest
import art_skills
from art_skills import SlnStrokeExpression
import gtsam
from gtsam.utils.test_case import GtsamTestCase
from gtsam.symbol_shorthand import X, P


class TestSlnStrokeExpression(GtsamTestCase):
    """Test SlnStrokeExpression wrapper."""

    def test_create_factor(self):
        """Test constructor and factor creation wrapping."""
        params_key = P(1)
        stroke = SlnStrokeExpression(params_key)
        factor = stroke.pos_integration_factor(12345, 0.01)


if __name__ == "__main__":
    unittest.main()
