"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit test for generative approach to strokes, using methodology described in:
Berio17eurographics_HandwritingTrajectories.pdf

Author: Juan-Diego Florez, Frank Dellaert, Sang-Won Leigh
"""
import math
# from os import stat
import unittest
import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import norm
from scipy import special
# from gtsam.utils.test_case import GtsamTestCase


class TrajectoryGenerator:
    """Top-level class."""

    def __init__(self, step_size, pts, delta, Ac, T):
        self.step_size = step_size  # timestep size (sec)
        self.pts = pts      # 2D array of target points, including origin
        self.delta = delta  # cent. angle (rad) of the stroke's circ. arc shape
        self.Ac = Ac  # shape parameter for log curve skewedness, from [0, 1]
        self.T = T    # period of the stroke (sec)

    def generate_stroke(self):
        """Use input values to generate a stroke"""
        stroke = StrokeGenerator()
        origin_point = stroke.origin_point(self.pts)
        sigma = stroke.sigma(self.Ac)
        mu = stroke.mu(sigma, self.T)
        D = stroke.D(self.pts)  # currently only works for 2 points
        theta = stroke.theta(self.pts)
        weights = stroke.weights(sigma, mu, self.step_size, self.T)
        displacements = stroke.displacements(D, theta, self.step_size,
                                             self.T, self.delta, weights)
        points = stroke.points(displacements, origin_point)
        return(points)

    def plot_trajectory(self):
        """Render trajectory plot."""


class StrokeGenerator:
    """Generates a single stroke."""

    # Define sigma and mu (response time & stroke delay both in log time-scale)
    # This comes from "Dijoua08EPM" and "Berio17_euro..."
    @staticmethod
    def origin_point(pts):
        return(pts[0])

    @staticmethod
    def sigma(Ac):
        """Sigma: log response time

        Args:
            Ac ([float]): [shape parameter that defines "skewdness" of the
                           lognormal curve]
        """
        return math.sqrt(-math.log(1 - Ac))

    @staticmethod
    def mu(sigma, T):
        """Mu: stroke delay

        Args:
            T ([float]): [period of the stroke]
        """
        return 3*sigma - math.log((-1 + math.exp(6*sigma))/T)

    @staticmethod
    def D(points):
        """D: stroke amplitude, distance between target points

        Args:
            points ([ndarray]): [2D array of two target points]
        """
        return(norm(points[1] - points[0]))

    @staticmethod
    def theta(points):
        """theta: angle between target points

        Args:
            points ([ndarray]): [2D array of two target points]
        """
        displacement = (points[1] - points[0])
        return(np.arctan(displacement[1]/displacement[0]))

    @staticmethod
    def weights(sigma, mu, step_size, T):
        """weights: time varying weight, used for displacement point generation

        Args:
            step_size ([float]): [timestep between generated points]
            T ([float]): [period of the stroke]
        """
        w = []
        steps = round(T/step_size)
        for t in range(1, steps+1):  # assign weight to each step
            time = t*step_size
            w_val = 0.5*(1 + special.erf((math.log10(time - 0) - mu)
                                         / (sigma*math.sqrt(2))))
            w.append(w_val)
        return(w)

    @staticmethod
    def displacements(D, theta, step_size, T, delta, weights):
        """displacements: weighted displacement at time (t) along stroke

        Args:
            step_size ([float]): [timestep between generated points]
            T ([float]): [period of the stroke]
            delta ([float]): [central angle of the circular arc shape of curve]
            weight ([list]): [weight at time t]
        """

        d = []
        steps = round(T/step_size)
        theta_0 = theta + (math.pi + delta)/2
        for t in range(0, steps-1):
            if norm(delta) > (1*10**(-10)):
                dx = D*((np.cos(theta_0 - delta*weights[t])
                         - np.cos(theta_0))*(2*np.sin(delta/2)) ** -1)
                dy = D*((np.sin(theta_0 - delta*weights[t])
                         - np.sin(theta_0))*(2*np.sin(delta/2)) ** -1)
            else:
                dx = D*(np.cos(theta)*weights[t])
                dy = D*(np.sin(theta)*weights[t])
            if t == 0:
                d = np.array([dx, dy])
            else:
                d_n = np.array([dx, dy])
                d = np.vstack((d, d_n))
        return(d)

    @staticmethod
    def points(displacements, origin_point):
        """points: points defining the stroke

        Args:
            _ ([]): []
        """
        px = origin_point[0]
        py = origin_point[1]
        fpoint = np.array([px, py])
        point = fpoint
        for i in range(len(displacements)):
            px = np.add(fpoint[0], displacements[i][0])
            py = np.add(fpoint[1], displacements[i][1])
            p = [px, py]
            point = np.vstack([point, p])
        return point


class RenderTrajectory:
    """Rendering the final plot."""

    def plot_geometry(self, pts, des_pts):
        """Plot the points with matplotlib"""
        x1, y1 = pts.T
        plt.scatter(x1, y1)
        xd, yd = des_pts.T
        plt.scatter(xd, yd)
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.title('Generated Strokes')
        plt.show()


class TestStrokeGenerator(unittest.TestCase):
    """Tests for the StrokeGenerator class."""

    def setUp(self):
        """Create class for testing multiple methods."""
        self.stroke_generator = StrokeGenerator()

    def test_origin_point(self):
        """Test the origin_point method"""
        pts = np.array([[0, 0], [3, 6]])
        np.testing.assert_array_equal(StrokeGenerator.origin_point(pts),
                                      [0, 0])
        return(StrokeGenerator.origin_point(pts))

    def test_sigma(self):
        """Test the sigma method."""
        Ac = 0.1
        # regression
        self.assertEqual(StrokeGenerator.sigma(Ac), 0.3245928459745012)
        return(StrokeGenerator.sigma(Ac))

    def test_mu(self):
        """Test the mu method."""
        sigma, T = self.test_sigma(), 0.7
        # regression
        self.assertEqual(StrokeGenerator.mu(sigma, T), -1.1765770263240554)
        return(StrokeGenerator.mu(sigma, T))

    def test_D(self):
        """Test the D method."""
        pts = np.array([[0, 0], [3, 6]])
        # regression
        self.assertEqual(StrokeGenerator.D(pts), 6.708203932499369)
        return(StrokeGenerator.D(pts))

    def test_theta(self):
        """Test the theta method."""
        pts = np.array([[0, 0], [3, 6]])
        # regression
        self.assertEqual(StrokeGenerator.theta(pts), 1.1071487177940904)
        return(StrokeGenerator.theta(pts))

    def test_weights_and_displacements(self):
        """Test the combined weights and displacements methods."""
        pts = np.array([[0, 0], [3, 6]])
        print('o', pts[1])
        sigma, mu, step_size, T = self.test_sigma(), self.test_mu(), 0.01, 0.7
        weight = StrokeGenerator.weights(sigma, mu, step_size, T)
        D, theta, delta = self.test_D(), self.test_theta(), 0.9
        # regression
        disps = StrokeGenerator.displacements(D, theta, step_size, T, delta,
                                              weight)
        np.testing.assert_array_almost_equal(disps[-1], pts[1], 1)
        return(disps)

    def test_points(self):
        """Test the points method."""
        print('d', len(self.test_weights_and_displacements()))
        self.assertEqual(len(self.test_weights_and_displacements())+1,
                         len(StrokeGenerator.points(
                                 self.test_weights_and_displacements(),
                                 self.test_origin_point())))


class Test_All(unittest.TestCase):
    """Test generative approach to lognormal curves for art skills project."""


if __name__ == "__main__":
    unittest.main()
