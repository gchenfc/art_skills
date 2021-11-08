from stroke_generator import StrokeGenerator
import numpy as np
import unittest


class TestStrokeGenerator(unittest.TestCase):
    """Tests for the StrokeGenerator class."""

    def setUp(self):
        """Create class for testing multiple methods."""
        self.stroke_generator = StrokeGenerator()

    def test_sigma(self):
        """Test the sigma method."""
        Ac = np.array([0.05, 0.05, 0.05])
        # regression, Berio code values
        np.testing.assert_array_almost_equal(StrokeGenerator.sigma(Ac),
                                             [0.22648023, 0.22648023,
                                              0.22648023])
        return(StrokeGenerator.sigma(Ac))

    def test_mu(self):
        """Test the mu method."""
        sigma, T = self.test_sigma(), 0.3
        # regression, Berio code values
        np.testing.assert_array_almost_equal(StrokeGenerator.mu(sigma, T),
                                             [-1.58642418, -1.58642418,
                                              -1.58642418])
        return(StrokeGenerator.mu(sigma, T))

    def test_D_theta(self):
        """Test the D and theta method."""
        t_points = np.array([[0, 0], [-50, 100], [100, 70], [-40, 120]]).T
        delta = np.array([0.3, 0.3, 0.3])
        D, D_adj, theta = StrokeGenerator.D_theta(StrokeGenerator(), t_points,
                                                  delta)
        # regression, Berio code values
        np.testing.assert_array_almost_equal(D_adj, [112.223765, 153.545734,
                                                     149.219632])
        np.testing.assert_array_almost_equal(theta, [2.034444, -0.197396,
                                                     2.798569])
        return(StrokeGenerator.D_theta(StrokeGenerator(), t_points, delta))

    def test_t0_t(self):
        """Test the t0 method."""
        dt = 0.01
        T, delta_t = 0.3, np.array([0.4, 0.4, 0.4])
        sigma, mu = self.test_sigma(), self.test_mu()
        t0, t = StrokeGenerator.t0_t(dt, sigma, mu, T, delta_t)
        # regression, Berio code values
        np.testing.assert_array_almost_equal(t0, [-0.10374027, 0.01625973, 
                                                  0.13625973],
                                             0.001)
        np.testing.assert_equal(len(t), 54)
        return(t0, t)

    def test_weight(self):
        """Test the weight method"""
        sigma, mu = self.test_sigma(), self.test_mu()
        t0, t = self.test_t0_t()
        weight = StrokeGenerator.weight(StrokeGenerator(), t, t0[0], sigma[0],
                                        mu[0])
        valid_w = True  # bool to check that w exists exclusively within [0,1]
        if any(t < 0 for t in weight) or any(t > 1 for t in weight):
            valid_w = False
        self.assertTrue(valid_w)  # Test weights are within [0, 1]
        self.assertEqual(len(weight), len(t))
        Berio_weight0 = np.array([0.0013499, 0.00474797, 0.0131555, 0.03015992,
                                  0.05937268, 0.1032772, 0.16234826,
                                  0.23480687, 0.317032,   0.40438253,
                                  0.4921001,  0.57603604, 0.65308165,
                                  0.7213026,  0.77985043, 0.82874624,
                                  0.8686204,  0.90046613, 0.92543827,
                                  0.94470749, 0.95936667, 0.97038008,
                                  0.9785636,  0.98458539, 0.98897853,
                                  0.99215926, 0.99444682, 0.99608233,
                                  0.99724561, 0.99806926, 0.9986501, 0.9990583,
                                  0.99934431, 0.99954419, 0.99968355,
                                  0.99978055, 0.99984794, 0.99989471,
                                  0.99992712, 0.99994958, 0.99996512,
                                  0.99997587, 0.9999833,  0.99998844, 0.999992,
                                  0.99999446, 0.99999616, 0.99999734,
                                  0.99999815, 0.99999872, 0.99999911,
                                  0.99999938, 0.99999957, 0.9999997])
        np.testing.assert_array_almost_equal(weight, Berio_weight0)
        return(weight)

    def test_displacement(self):
        """Test the stroke method"""
        t_points = np.array([[0, 0], [-50, 100], [100, 70], [-40, 120]]).T
        weight = self.test_weight()
        D, D_adj, theta = self.test_D_theta()
        delta = np.array([0.3, 0.3, 0.3])
        displacement = StrokeGenerator.displacement(StrokeGenerator(),
                                                    t_points, weight, D[0],
                                                    theta[0], delta[0])
        np.testing.assert_array_almost_equal([displacement[0][-1],
                                              displacement[1][-1]],
                                             [t_points[0][1],
                                              t_points[1][1]], 3)
        return(displacement)

    def test_trajectory(self):
        """Test trajectory generation"""
        t_points = np.array([[0, 0], [-50, 100], [100, 70], [-40, 120]]).T
        sigma, mu = self.test_sigma(), self.test_mu()
        D, D_adj, theta = self.test_D_theta()
        t0, t = self.test_t0_t()
        delta = np.array([0.3, 0.3, 0.3])
        trajectory = np.zeros((2, 54))
        for i in range(len(t_points[0]) - 1):
            weight = StrokeGenerator.weight(StrokeGenerator(), t, t0[i],
                                            sigma[i], mu[i])
            displacement = StrokeGenerator.displacement(StrokeGenerator(),
                                                        t_points, weight, D[i],
                                                        theta[i], delta[i])
            trajectory[:, :] += displacement
        np.testing.assert_allclose([trajectory[0][-1],
                                    trajectory[1][-1]],
                                   [t_points[0][-1],
                                    t_points[1][-1]], 0.1)


if __name__ == "__main__":
    unittest.main()