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

    def __init__(self, step_size, t_points, delta, Ac, T):
        self.step_size = step_size  # timestep size (sec)
        self.t_points = t_points  # 2D array of target points, including origin
        self.delta = delta  # cent. angle (rad) of the stroke's circ. arc shape
        self.Ac = Ac  # shape parameter for log curve skewedness, from [0, 1]
        self.T = T    # period of the stroke (sec)

    def generate_strokes(self):
        """Use input values to generate a stroke"""
        stroke = StrokeGenerator()
        sigma = stroke.sigma(self.Ac)
        mu = stroke.mu(sigma, self.T)
        D_scalar = stroke.D_scalar(self.t_points)
        theta = stroke.theta(self.t_points)
        weights = stroke.weights(self.step_size, self.T, sigma, mu)
        displacements = stroke.displacements(self.step_size, self.T, D_scalar,
                                             theta, self.delta, weights)
        points = stroke.points(displacements, self.t_points)
        extracted_velocities = stroke.extract_velocity(self.step_size, points)
        return(points, extracted_velocities)


class StrokeGenerator:
    """Generates one stroke from trajectory input."""

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
    def D_scalar(t_points):
        """D: stroke amplitude, distance between target points

        Args:
            t_points ([ndarray]): [2D array of two target points]
        """
        D = [norm(t_points[x] - t_points[x-1])
             for x in range(1, len(t_points))]
        return(D)

    @staticmethod
    def D(t_points):
        """D: stroke amplitude, distance between target points

        Args:
            t_points ([ndarray]): [2D array of two target points]
        """
        D = [t_points[x] - t_points[x-1]
             for x in range(1, len(t_points))]
        return(D)

    @staticmethod
    def theta(t_points):
        """theta: angle between target points

        Args:
            t_points ([ndarray]): [2D array of two target points]
        """
        theta = [np.arctan2(t_points[x][1] - t_points[x-1][1],
                 t_points[x][0] - t_points[x-1][0])
                 for x in range(1, len(t_points))]
        return(theta)

    # TODO add t0 to this function
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
        t0 = 0  # temporary fix
        relative_times = [[round((x*step_size), 2)
                           for x in range(1, steps[y]+1)]
                          for y in range(len(steps))]
        weight = [[(0.5*(1 + special.erf((math.log(t - t0) - y) /
                    (x*math.sqrt(2))))) for t in relative_times[z]]
                  for x, y, z in zip(sigma, mu, range(len(T)))]
        return(weight)

    @staticmethod
    def displacements(step_size, T, D_scalar, theta, delta, weight):
        """displacements: weighted displacement at time (t) along stroke

        Args:
            step_size ([float]): [timestep between generated points]
            T ([float]): [period of the stroke]
            delta ([float]): [central angle of the circular arc shape of curve]
            weight ([list]): [weight at time t]
        """
        T = [round(t, 2) for t in T]
        theta_0 = [(x + (math.pi + y)/2) for x, y in zip(theta, delta)]
        steps = [round(x/step_size) for x in T]
        eps = 1*10**(-10)  # a really small value, used to avoid divide by zero
        dx = [[(D_scalar[i]*((np.cos(x - y*weight[i][t]) - np.cos(x))
                             * (2*np.sin(y/2))**-1)) if norm(y) > eps
               else (D_scalar[i]*(np.cos(j)*weight[i][t])) for t in range(z-1)]
              for x, y, z, i, j in
              zip(theta_0, delta, steps, range(len(weight)), theta)]
        dy = [[(D_scalar[i]*((np.sin(x - y*weight[i][t]) - np.sin(x))
                             * (2*np.sin(y/2))**-1)) if norm(y) > eps
               else (D_scalar[i]*(np.sin(j)*weight[i][t])) for t in range(z-1)]
              for x, y, z, i, j in
              zip(theta_0, delta, steps, range(len(weight)), theta)]
        print(dx[0][-1])
        displacement = [np.column_stack((dx[x], dy[x]))
                        for x in range(len(dx))]
        # print("\n")
        # print(type(displacement))
        # print(type(displacement[0]))
        # print(displacement, "\n")
        # print(displacement[0], "\n")
        # print(displacement[1])
        return(displacement)

    @staticmethod
    def points(displacement, t_points):
        """points: points defining the stroke

        Args:
            _ ([]): []
        """
        for x in range(len(displacement)):
            fpoint = np.array([t_points[x][0], t_points[x][1]])
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

    @staticmethod
    def extract_velocity(step_size, points):
        velocities = [math.dist(points[x], points[x-1])/step_size
                      for x in range(1, len(points))]
        time = [round(step_size * x, 2) for x in range(1, len(points))]
        velocity = [velocities, time]
        return(velocity)

    # TODO add t0 to this function
    # def SL_velocity(step_size, T, mu, sigma, D):
    #     steps = [round(x/step_size) for x in T]
    #     t0 = 0  # temporary fix
    #     relative_times = [[round((x*step_size), 2)
    #                        for x in range(1, steps[y]+1)]
    #                       for y in range(len(steps))]
    #     lambda_i = [[(1/(sigma*math.sqrt(2*math.pi)*(t - t0)) *
    #                   math.exp(((math.log(t - t0) - mu)**2)/(2*sigma**2)))
    #                  for t in relative_times[z]]
    #                 for x, y, z in zip(sigma, mu, range(len(T)))]
    #     vel =


class TestStrokeGenerator(unittest.TestCase):
    """Tests for the StrokeGenerator class."""

    def setUp(self):
        """Create class for testing multiple methods."""
        self.stroke_generator = StrokeGenerator()

    def test_sigma(self):
        """Test the sigma method."""
        Ac = [0.043949, 0.1082468]
        # regression
        np.testing.assert_array_almost_equal(StrokeGenerator.sigma(Ac),
                                             [0.212, 0.338476])
        return(StrokeGenerator.sigma(Ac))

    def test_mu(self):
        """Test the mu method."""
        sigma, T = self.test_sigma(), [0.3, 0.4]
        # regression
        np.testing.assert_array_almost_equal(StrokeGenerator.mu(sigma, T),
                                             [-1.511093, -1.791049])
        return(StrokeGenerator.mu(sigma, T))

    def test_D_scalar(self):
        """Test the D method."""
        t_points = np.array([[25, 18], [6, 4], [8, 5.5]])
        np.testing.assert_array_almost_equal(StrokeGenerator.D_scalar(t_points),
                                             [23.600847, 2.5])
        return(StrokeGenerator.D(t_points))

    def test_theta(self):
        """Test the theta method."""
        t_points = np.array([[25, 18], [6, 4], [8, 5.5]])
        # regression
        np.testing.assert_array_almost_equal(StrokeGenerator.theta(t_points),
                                             [-2.506566, 0.643501])
        return(StrokeGenerator.theta(t_points))

    def test_weights_and_displacements(self):
        """Test the combined weights and displacements methods."""
        step_size, T = 0.01, [0.260832, 0.150658]
        sigma, mu = self.test_sigma(), self.test_mu()
        weight = StrokeGenerator.weights(step_size, T, sigma, mu)
        valid_w = True  # bool to check that w exists exclusively within [0,1]
        for i in range(0, len(weight)):
            if any(t < 0 for t in weight[i]) or any(t > 1 for t in weight[i]):
                valid_w = False
        self.assertTrue(valid_w)  # Test weights are within [0, 1]
    #     t_points = np.array([[25,18], [6,4], [8,5.5]])
    #     D_scalar, theta = self.test_D_scalar(), self.test_theta()
    #     delta = [-0.001, 0.001]
    #     displacement = StrokeGenerator.displacements(step_size, T, D_scalar,
    #                                                  theta, delta, weight)
    #     print(displacement)
    #     np.testing.assert_array_almost_equal(displacement[-1][-1],
    #                                          t_points[-1] - t_points[-2], 0.1)
    #     return(displacement)

    # def test_points(self):
    #     """Test the points method."""
    #     t_points = np.array([[0, 0], [15, 9], [2, 11]])
    #     points = StrokeGenerator.points(self.test_weights_and_displacements(),
    #                                     t_points)
    #     np.testing.assert_array_almost_equal(points[-1],
    #                                           t_points[-1], 0.01)
    #     return points

    # def test_velocity(self):
    #     """Test the velocity method."""
    #     step_size = 0.01
    #     velocity = StrokeGenerator.extract_velocity(step_size,
    #                                                 self.test_points())
    #     self.assertEqual(len(velocity[0]) + 1, len(self.test_points()))


if __name__ == "__main__":
    unittest.main()
