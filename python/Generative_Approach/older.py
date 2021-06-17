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
import unittest
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
        sigma = stroke.sigma(self.Ac)
        mu = stroke.mu(sigma, self.T)
        D = stroke.D(self.pts)  # currently only works for 2 points
        theta = stroke.theta(self.pts)
        weights = stroke.weights(self.step_size, self.T, sigma, mu)
        displacements = stroke.displacements(self.step_size, self.T, D, theta,
                                             self.delta, weights)
        points = stroke.points(displacements, self.pts)
        return(points)


class StrokeGenerator:
    """Generates a single stroke."""

    # Define sigma and mu (response time & stroke delay both in log time-scale)
    # This comes from "Dijoua08EPM" and "Berio17_euro..."
    @staticmethod
    def sigma(Ac):
        """Sigma: log response time (the logresponse times, the response times
        of the neuromuscular systems expressed on a logarithmic time scale)

        Args:
            Ac ([float]): [shape parameter that defines "skewdness" of the
                           lognormal curve]
        """
        sigma = [math.sqrt(-math.log(1 - x)) for x in Ac]
        return(sigma)

    @staticmethod
    def mu(sigma, T):
        """Mu: stroke delay (the logtime delays, the time delays of the
        neuromuscular systems expressed on a logarithmic time scale)

        Args:
            T ([float]): [period of the stroke]
        """
        mu = [(3*x - math.log((-1 + math.exp(6*x))/y))
              for x, y in zip(sigma, T)]
        return mu

    @staticmethod
    def D(points):
        """D: stroke amplitude, distance between target points

        Args:
            points ([ndarray]): [2D array of two target points]
        """
        D = [norm(points[x] - points[x-1]) for x in range(1, len(points))]
        return(D)

    @staticmethod
    def theta(points):
        """theta: angle between target points

        Args:
            points ([ndarray]): [2D array of two target points]
        """
        theta = [np.arctan2(points[x][1] - points[x-1][1],
                 points[x][0] - points[x-1][0])
                 for x in range(1, len(points))]
        return(theta)

    @staticmethod
    def weights(step_size, T, sigma, mu):
        """weights: time varying weight, used for displacement point generation

        Args:
            step_size ([float]): [timestep between generated points]
            T ([float]): [period of the stroke]
            sigma
            mu
        """
        steps = [round(x/step_size) for x in T]
        time = [[round((x*step_size), 2) for x in range(1, steps[y]+1)]
                for y in range(len(steps))]
        weight = [[(0.5*(1 + special.erf((math.log(t - 0) - y) /
                    (x*math.sqrt(2))))) for t in time[z]]
                  for x, y, z in zip(sigma, mu, range(len(sigma)))]
        return(weight)

    @staticmethod
    def displacements(step_size, T, D, theta, delta, weight):
        """displacements: weighted displacement at time (t) along stroke

        Args:
            step_size ([float]): [timestep between generated points]
            T ([float]): [period of the stroke]
            delta ([float]): [central angle of the circular arc shape of curve]
            weight ([list]): [weight at time t]
        """
        theta_0 = [(x + (math.pi + y)/2) for x, y in zip(theta, delta)]
        steps = [round(x/step_size) for x in T]
        eps = 1*10**(-10)  # a really small value, used to avoid divide by zero
        dx = [[(D[i]*((np.cos(x - y*weight[i][t]) - np.cos(x))
                      * (2*np.sin(y/2))**-1)) if norm(y) > eps
               else (D[i]*(np.cos(j)*weight[i][t])) for t in range(z-1)]
              for x, y, z, i, j in
              zip(theta_0, delta, steps, range(len(weight)), theta)]
        dy = [[(D[i]*((np.sin(x - y*weight[i][t]) - np.sin(x))
                      * (2*np.sin(y/2))**-1)) if norm(y) > eps
               else (D[i]*(np.sin(j)*weight[i][t])) for t in range(z-1)]
              for x, y, z, i, j in
              zip(theta_0, delta, steps, range(len(weight)), theta)]
        # using temp variables to get 2d array of displacement points
        displacements_temp1 = [np.column_stack((dx[x], dy[x]))
                               for x in range(len(dx))]
        # displacements_temp2 = [np.vstack((displacements_temp1[x-1],
        #                        displacements_temp1[x]))
        #                        for x in range(1, len(displacements_temp1))]
        displacement = displacements_temp1
        return(displacement)

    @staticmethod
    def points(displacement, pts):
        """points: points defining the stroke

        Args:
            _ ([]): []
        """
        for x in range(len(displacement)):
            fpoint = np.array([pts[x][0], pts[x][1]])
            P = fpoint
            if x == 0:
                points = fpoint
            for i in range(len(displacement[x])):
                px = np.add(fpoint[0], displacement[x][i][0])
                py = np.add(fpoint[1], displacement[x][i][1])
                p = [px, py]
                P = np.vstack([P, p])
            points = np.vstack([points, P])
        return(points)


class TestStrokeGenerator(unittest.TestCase):
    """Tests for the StrokeGenerator class."""

    def setUp(self):
        """Create class for testing multiple methods."""
        self.stroke_generator = StrokeGenerator()

    def test_sigma(self):
        """Test the sigma method."""
        Ac = [0.1, 0.2]
        # regression
        np.testing.assert_array_almost_equal(StrokeGenerator.sigma(Ac),
                                             [0.324593, 0.472381])
        return(StrokeGenerator.sigma(Ac))

    def test_mu(self):
        """Test the mu method."""
        sigma, T = self.test_sigma(), [0.5, 0.7]
        # regression
        np.testing.assert_array_almost_equal(StrokeGenerator.mu(sigma, T),
                                             [-1.513049, -1.713259])
        return(StrokeGenerator.mu(sigma, T))

    def test_D(self):
        """Test the D method."""
        pts = np.array([[0, 0], [3, 6], [5, 10]])
        np.testing.assert_array_almost_equal(StrokeGenerator.D(pts),
                                             [6.708204, 4.472136])
        return(StrokeGenerator.D(pts))

    def test_theta(self):
        """Test the theta method."""
        pts = np.array([[0, 0], [3, 6], [5, 10]])
        # regression
        np.testing.assert_array_almost_equal(StrokeGenerator.theta(pts),
                                             [1.107149, 1.107149])
        return(StrokeGenerator.theta(pts))

    def test_weights_and_displacements(self):
        """Test the combined weights and displacements methods."""
        pts = np.array([[0, 0], [3, 6], [5, 10]])
        step_size, T = 0.01, [0.5, 0.7]
        sigma, mu = self.test_sigma(), self.test_mu()
        weight = StrokeGenerator.weights(step_size, T, sigma, mu)
        valid_w = True  # bool to check that w exists exclusively within [0,1]
        for i in range(0, len(weight)):
            if any(t < 0 for t in weight[i]) or any(t > 1 for t in weight[i]):
                valid_w = False
        self.assertTrue(valid_w)  # Test weights are within [0, 1] (Success!)

        D, theta, delta = self.test_D(), self.test_theta(), [0.9, 0.3]
        displacement = StrokeGenerator.displacements(step_size, T, D, theta,
                                                     delta, weight)
        print(displacement[-1][-1])
        print(pts[-1] - pts[-2])
        np.testing.assert_array_almost_equal(displacement[-1][-1],
                                             pts[-1] - pts[-2], decimal=2)
        return(displacement)

    def test_points(self):
        """Test the points method."""
        pts = np.array([[0, 0], [3, 6], [5, 10]])
        points = StrokeGenerator.points(self.test_weights_and_displacements(),
                                        pts)
        np.testing.assert_array_almost_equal(points[-1],
                                             pts[-1], 0.1)


if __name__ == "__main__":
    unittest.main()
