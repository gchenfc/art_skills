from trajectory_generator import TrajectoryGenerator
import numpy as np
import unittest


class TestTrajectoryGenerator(unittest.TestCase):
    """Tests for the TrajectoryGenerator class."""

    def setUp(self):
        """Create class for testing multiple methods."""
        self.trajectory_generator = TrajectoryGenerator()

    def test_XX(self):
        """Test the sigma method."""
        return(TrajectoryGenerator.XX)


if __name__ == "__main__":
    unittest.main()
    