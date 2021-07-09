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

    def __init__(self, step_size, t_points, delta, Ac, delta_t, T):
        self.step_size = step_size  # timestep size (sec)
        self.t_points = t_points  # 2D array of target points, including origin
        self.delta = delta  # cent. angle (rad) of the stroke's circ. arc shape
        self.Ac = Ac  # shape parameter for log curve skewedness, from [0, 1]
        self.delta_t = delta_t  # rel. t-offset wrt prev. stroke occu. & durat.
        self.T = T    # period of the stroke (sec)

    def generate_strokes(self):
        """Use input values to generate a stroke"""
        stroke = StrokeGenerator()
        sigma = stroke.sigma(self.Ac)
        mu = stroke.mu(sigma, self.T)
        D = stroke.D(self.t_points)
        theta = stroke.theta(self.t_points)
        t0 = stroke.t0_i(sigma, mu, self.delta_t, D)
        weights = stroke.weights(self.step_size, self.T, sigma, mu, t0)
        displacements = stroke.displacements(self.step_size, self.T, D,
                                             theta, self.delta, weights)
        points = stroke.points(displacements, self.t_points)
        velocities = stroke.velocity(self.step_size, points)
        vel = stroke.SL_velocity(self.step_size, self.T, mu, sigma, D, t0)
        return(points, velocities, vel, weights, displacements)


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
        sigma = [math.sqrt(-math.log(1.0 - x)) for x in Ac]
        return(sigma)

    @staticmethod
    def mu(sigma, T):
        """Mu: stroke delay (the logtime delays, the time delays of the
        neuromuscular systems expressed on a logarithmic time scale)

        Args:
            T ([float]): [period of the stroke]
        """
        mu = [(3*s - math.log((-1.0 + math.exp(6*s))/t))
              for s, t in zip(sigma, T)]
        return(mu)

    @staticmethod
    def D(t_points):
        """D: stroke amplitude, distance between target points

        Args:
            t_points ([ndarray]): [2D array of two target points]
        """
        D = [norm(t_points[x] - t_points[x-1])
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

    @staticmethod
    def t1_i(delta_t, D):
        """t1: onset time of the lognormal stroke profile

        Args:
            T_prev: the period of the pervious stroke
            d_prev: displacement of the previous stroke
        """

        t1 = np.empty([1, len(delta_t)])
        for n in range(len(delta_t)):
            if n < 1:
                t1[0][n] = 0
            else:
                t1[0][n] = t1[0][n-1] + D[n-1]*delta_t[n]
        return(t1[0])

    def t0_i(self, sigma, mu, delta_t, D):
        """t0: the time of ocurrence of the command that the lognormal impulse
               responds to
                    D(t)i describes the direction and amplitude of each stroke
                    (towards a virtual target) and describes the lognormal
                    impulse response of each stroke to a centrally generated
                    command occurring at time t0i.
        Args:
            
        """
        t1 = self.t1_i(delta_t, D)
        t0 = [(t1[n] - math.exp(mu[n] - sigma[n])) for n in range(len(t1))]
        t0[0] = 0
        return(t0)

    @staticmethod
    def t0_i_thesis(sigma, delta_t):
        """t0: the time of ocurrence of the command that the lognormal impulse
               responds to
                    D(t)i describes the direction and amplitude of each stroke
                    (towards a virtual target) and describes the lognormal
                    impulse response of each stroke to a centrally generated
                    command occurring at time t0i.
        Args:

        """
        t0 = np.empty([1, 2])

        for n in range(len(sigma)):
            if n == 0:
                t0[0][n] = 0
            else:
                t0[0][n] = (t0[0][n-1] + delta_t[n]*math.sinh(3*sigma[n]))
        return(t0[0])

    @staticmethod
    def weights(step_size, T, sigma, mu, t0):
        """weights: time varying weight, used for displacement point generation

        Args:
            step_size ([float]): [timestep between generated points]
            T ([float]): [period of the stroke]
            sigma
            mu
        """

        # t = t - t0
        # t = np.maximum(t, log_eps) #1E-200)
        # return 0.5*(1 + erf( (np.log(t) - mu) / (np.sqrt(2)*sigma) ))

        steps = [round(x/step_size) for x in T]

        times = [[round((x*step_size), 2)
                           for x in range(1, steps[y]+1)]
                          for y in range(len(steps))]

        for i in range(len(times)):
            if i > 0:
                for n in range(len(times[i])):
                    times[i][n] = round((times[i-1][-1] + times[i][n]), 2)
        
        weight = [[(0.5*(1 + special.erf((math.log(t - t0[n]) - y)
        # weight = [[(0.5*(1 + special.erf((math.log(t) - y)
                                         / (x*math.sqrt(2)))))
                   for t in times[n]]
                  for x, y, n in zip(sigma, mu, range(len(T)))]
        print(weight)
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
        T = [round(t, 2) for t in T]
        # theta_0 = [(x + (math.pi + y)/2) for x, y in zip(theta, delta)]
        theta_0 = [(x + (math.pi + y)/2) for x, y in zip(theta, delta)]
        steps = [round(x/step_size) for x in T]
        eps = 1e-15  # a really small value, used to avoid divide by zero
        dx = [[(D[i]*((np.cos(x - y*weight[i][t]) - np.cos(x)) /
                      (2*np.sin(y/2)))) if norm(y) > eps
               else (D[i]*(np.cos(j)*weight[i][t])) for t in range(z-1)]
              for x, y, z, i, j in
              zip(theta_0, delta, steps, range(len(weight)), theta)]
        dy = [[(D[i]*((np.sin(x - y*weight[i][t]) - np.sin(x)) /
                      (2*np.sin(y/2)))) if norm(y) > eps
               else (D[i]*(np.sin(j)*weight[i][t])) for t in range(z-1)]
              for x, y, z, i, j in
              zip(theta_0, delta, steps, range(len(weight)), theta)]
        displacement = [np.column_stack((dx[x], dy[x]))
                        for x in range(len(dx))]
        return(displacement)

    @staticmethod
    def points(displacement, t_points):
        """points: points defining the stroke

        Args:
            _ ([]): []
        """
        trajectory = []
        for x in range(len(displacement)):  # number of strokes
            if x == 0:
                p0 = np.array([t_points[0][0], t_points[0][1]])
                stroke = p0
            else:
                p0 = stroke[-1]
            for i in range(len(displacement[x])):  # number of points
                px = np.add(p0[0], displacement[x][i][0])
                py = np.add(p0[1], displacement[x][i][1])
                p = [px, py]
                stroke = np.vstack([stroke, p])
            # points = np.vstack([points, stroke])
            trajectory.append(stroke)
        return(trajectory)

    @staticmethod
    def velocity(step_size, points):
        velocity_profiles = []
        T = []
        for i in range(len(points)):
            velocity_prof = [math.dist(points[i][x], points[i][x-1])/step_size
                             for x in range(1, len(points[i]))]
            time = [round(step_size * x, 2) if i == 0
                    else (round(step_size * x, 2))  # + T[-1])
                    for x in range(1, len(points[i]))]
            T = time
            velocity_profiles.append([velocity_prof, time])
        return(velocity_profiles)

    @staticmethod
    def SL_velocity(step_size, T, mu, sigma, D, t0):
        steps = [round(t/step_size) for t in T]
        relative_times = [[round((x*step_size), 2)
                           for x in range(1, steps[y]+1)]
                          for y in range(len(steps))]
        lambda_i = [[(1/(x*math.sqrt(2*math.pi)*(t - t0[n])) *
                      math.exp(((math.log(t - t0[n]) - y)**2)/(2*x**2)))
                     for t in relative_times[n]]
                    for x, y, n in zip(sigma, mu, range(len(T)))]
        vel = [[D[n]*lambda_i[n][t] for t in range(len(lambda_i[n]))]
               for n in range(len(T))]
        V = [vel, relative_times]
        return(V)


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
        t_points = np.array([[0, 0], [3, 6], [5, 10]])
        D = StrokeGenerator.D(t_points)
        np.testing.assert_array_almost_equal(D, [6.708204, 4.472136])
        return(StrokeGenerator.D(t_points))

    def test_theta(self):
        """Test the theta method."""
        t_points = np.array([[0, 0], [3, 6], [5, 10]])
        # regression
        np.testing.assert_array_almost_equal(StrokeGenerator.theta(t_points),
                                             [1.107149, 1.107149])
        return(StrokeGenerator.theta(t_points))

    def test_t1_t0(self):
        """Test the t1 method."""
        delta_t = [0, -0.05]
        sigma, mu, D = self.test_sigma(), self.test_mu(), self.test_D()
        np.testing.assert_array_almost_equal(StrokeGenerator.t1_i(delta_t, D),
                                             [0, -0.33541])
        """Test the t0 method."""
        t0 = StrokeGenerator.t0_i(StrokeGenerator, sigma, mu, delta_t, D)
        np.testing.assert_array_almost_equal(t0, [0, -0.447816])

        t0i = StrokeGenerator.t0_i_thesis(sigma, delta_t)

        #print("t0i", t0i, "t0", t0)
        return(t0i)

    def test_weights_and_displacements(self):
        """Test the weights method"""
        step_size, T = 0.01, [0.5, 0.7]
        sigma, mu, t0 = self.test_sigma(), self.test_mu(), self.test_t1_t0()
        weight = StrokeGenerator.weights(step_size, T, sigma, mu, t0)
        valid_w = True  # bool to check that w exists exclusively within [0,1]
        for i in range(0, len(weight)):
            if any(t < 0 for t in weight[i]) or any(t > 1 for t in weight[i]):
                valid_w = False
        self.assertTrue(valid_w)  # Test weights are within [0, 1]
        """Test the displacement method"""
        t_points = np.array([[0, 0], [3, 6], [5, 10]]).astype(float)
        D, theta = self.test_D(), self.test_theta()
        delta = [0.9, 0.3]
        displacement = StrokeGenerator.displacements(step_size, T, D, theta,
                                                     delta, weight)
        np.testing.assert_array_almost_equal(displacement[-1][-1],
                                             t_points[-1] - t_points[-2], 0.1)
        
        return(displacement)

    def test_points(self):
        """Test the points method."""
        t_points = np.array([[0, 0], [3, 6], [5, 10]])
        points = StrokeGenerator.points(self.test_weights_and_displacements(),
                                        t_points)
        np.testing.assert_array_almost_equal(points[-1][-1],
                                             t_points[-1], 0.01)
        return points

    # def test_velocity(self):
        # """Test the velocity method."""
        # step_size = 0.01
        # velocity = StrokeGenerator.velocity(step_size,
        #                                     self.test_points())
        # self.assertEqual(len(velocity), len(self.test_points()))


if __name__ == "__main__":
    unittest.main()
