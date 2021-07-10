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
from scipy import special
# from scipy.stats import lognorm
# from gtsam.utils.test_case import GtsamTestCase


class TrajectoryGenerator:
    """Top-level class."""

    def __init__(self, dt, t_points, delta, Ac, delta_t, T):
        self.dt = dt  # timestep size (sec)
        self.t_points = t_points  # 2D array of target points, including origin
        self.delta = delta  # cent. angle (rad) of the stroke's circ. arc shape
        self.Ac = Ac  # shape parameter for log curve skewedness, from [0, 1]
        self.delta_t = delta_t  # rel. t-offset wrt prev. stroke occu. & durat.
        self.T = T    # period of the stroke (sec)

    def trajectory_setup(self):
        """Use input values to setup all necessary derived parameters"""
        strokegen = StrokeGenerator()
        sigma = strokegen.sigma(self.Ac)
        mu = strokegen.mu(sigma, self.T)
        D, D_adj, theta = strokegen.D_theta(self.t_points, self.delta)
        t0, t = strokegen.t0_t(self.dt, sigma, mu, self.T, self.delta_t)
        return(sigma, mu, D, theta, t0, t)

    def generate_stroke(self, t, t0, sigma, mu, D, theta, delta):
        """Use input values to generate a stroke"""
        strokegen = StrokeGenerator()
        weight = strokegen.weight(t, t0, sigma, mu)
        displacement = strokegen.displacement(self.t_points, weight, D, theta,
                                              delta)
        # velocities = strokegen.velocity(self.dt, points)
        # vel = strokegen.SL_velocity(self.dt, self.T, mu, sigma, D, t0)
        return(displacement)

    def generate_trajectory(self):
        sigma, mu, D, theta, t0, t = self.trajectory_setup()
        trajectory = np.zeros((2, 54))
        for i in range(len(self.t_points[0]) - 1):
            displacement = self.generate_stroke(t, t0[i], sigma[i], mu[i],
                                                D[i], theta[i], self.delta[i])
            trajectory[:, :] += displacement
        return(trajectory)

    def velocity(self):
        trajectory = self.generate_trajectory()
        velocity = np.sqrt(np.sum((np.gradient(trajectory, axis=1)/self.dt)**2,
                                  axis=0))
        return(velocity)


class StrokeGenerator:
    """Generates strokes from trajectory input."""

    def __init__(self, eps=1e-15, sensitivity=1e-10):
        self.eps = eps  # a really small value, used to avoid divide by zero
        self.sensitivity = sensitivity  # a small value used for filtering
    """Trajectory Level Methods: (operate on all input values)"""
    @staticmethod
    def sigma(Ac):
        """
        Sigma: log response time (the logresponse times, the response times
        of the neuromuscular systems expressed on a logarithmic time scale)

        Args:
            Ac ([float]): [shape parameter that defines "skewdness" of the
                           lognormal curve]
        """
        sigma = np.sqrt(-np.log(1.0 - Ac))
        return(sigma)

    @staticmethod
    def mu(sigma, T):
        """
        Mu: stroke delay (the logtime delays, the time delays of the
        neuromuscular systems expressed on a logarithmic time scale)

        Args:
            T ([float]): [period of the stroke]
        """
        mu = 3.*sigma - np.log((-1.0 + np.exp(6.*sigma))/T)
        return(mu)

    def D_theta(self, t_points, delta):
        """D: stroke amplitude, distance between target points
        theta: angle between target points

        Args:
            t_points ([ndarray]): [2D array of two target points]
        """
        t_disp = np.diff(t_points, axis=1)
        D = np.sqrt(t_disp[0, :]**2 + t_disp[1, :]**2)
        theta = np.arctan2(t_disp[1, :], t_disp[0, :])
        # shift D with lognormal (using delta parameter):
        h = (delta/2) / np.sin(delta/2)
        h[np.abs(np.sin(delta)) < self.sensitivity] = 1.
        D_adj = D*h
        return(D, D_adj, theta)

    @staticmethod
    def t0_t(dt, sigma, mu, T, delta_t):
        """
        t0: the time of ocurrence of the command that the lognormal impulse
               responds to
                    D(t)i describes the direction and amplitude of each stroke
                    (towards a virtual target) and describes the lognormal
                    impulse response of each stroke to a centrally generated
                    command occurring at time t0i.

        Args:
            T_prev: the period of the pervious stroke
            d_prev: displacement of the previous stroke
        """
        n = delta_t.shape[0]

        """Method from Berio code, similar to Berio 17"""
        # add small time offset at the start to guarantee that
        # the first stroke starts with zero velocity
        # t_offset = 0.05
        t0 = np.zeros(n)  # + t_offset
        """eqtn: t0 = t0_(i-1) + delta_t*T - e^(mu - sigma)"""
        t0[1:] = delta_t[1:] * T[1:]
        t0 = np.cumsum(t0)
        # Add onsets in order to shift lognormal to start
        t0 = t0 - np.exp(mu[0] - sigma[0]*3)
        endtime = t0[-1] + np.exp(mu[-1] + sigma[-1]*3)
        t = np.arange(0.0, endtime, dt)
        return(t0, t)

    """Stroke Level Methods: (operate on stroke-specific input values)"""
    def weight(self, t, t0, sigma, mu):
        """weight: lognormal interpolation between target points

        Args:
            t ([float]): [time domain of trajectory]
            t0 ([float]): []
            sigma
            mu
        """
        t_stroke = np.maximum((t - t0), self.eps)
        weight = 0.5*(1 + special.erf((np.log(t_stroke) - mu) /
                                      (sigma*math.sqrt(2))))
        return(weight)

    def displacement(self, t_points, weight, D, theta, delta):
        """displacement: discrete trajectory between 2 target points

        Args:
            dt ([float]): [timestep between generated points]
            T ([float]): [period of the stroke]
            delta ([float]): [central angle of the circular arc shape of curve]
            weight ([list]): [weight at time t]
        """
        P0 = t_points.T[0]
        P0 = P0.reshape(-1, 1)
        P0_offset = np.ones((2, len(weight)))*P0
        # in Berio17, this is "theta + ..."
        theta_0 = theta - (math.pi + delta)/2
        # if abs(delta) > self.eps:  # eps according to Berio17
        if abs(delta) > self.sensitivity:  # "... > sensitivity in Berio code
            # in Berio17: trig_fn(th0 - delta*weight) - ...
            d = D*np.vstack([(np.cos(theta_0 + delta*weight) -
                              np.cos(theta_0)) / (2*np.sin(delta/2)),
                             (np.sin(theta_0 + delta*weight) -
                              np.sin(theta_0)) / (2*np.sin(delta/2))])
            displacement = d + P0_offset
        else:
            d = D * np.vstack([np.cos(theta)*weight,
                               np.sin(theta)*weight])
            displacement = d + P0_offset
        return(displacement)

    @staticmethod
    def velocity(dt, points):
        velocity_profiles = []
        for i in range(len(points)):
            velocity_prof = [math.dist(points[i][x], points[i][x-1])/dt
                             for x in range(1, len(points[i]))]
            time = [round(dt * x, 2) if i == 0
                    else (round(dt * x, 2))  # + T[-1])
                    for x in range(1, len(points[i]))]
            velocity_profiles.append([velocity_prof, time])
        return(velocity_profiles)

    @staticmethod
    def SL_velocity(dt, T, mu, sigma, D, t0):
        steps = [round(t/dt) for t in T]
        relative_times = [[round((x*dt), 2)
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
        Ac = np.array([0.05, 0.05, 0.05])
        # regression, Berio code values
        np.testing.assert_array_almost_equal(StrokeGenerator.sigma(Ac),
                                             [0.22648023, 0.22648023,
                                              0.22648023])
        return(StrokeGenerator.sigma(Ac))

    def test_mu(self):
        """Test the mu method."""
        sigma, T = self.test_sigma(), np.array([0.3, 0.3, 0.3])
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
        T, delta_t = np.array([0.3, 0.3, 0.3]), np.array([0.4, 0.4, 0.4])
        sigma, mu = self.test_sigma(), self.test_mu()
        t0, t = StrokeGenerator.t0_t(dt, sigma, mu, T, delta_t)
        # regression, Berio code values
        np.testing.assert_array_almost_equal(t0, [0., 0.115824, 0.235824],
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

    # def test_velocity(self):
        # """Test the velocity method."""
        # dt = 0.01
        # velocity = StrokeGenerator.velocity(dt,
        #                                     self.test_points())
        # self.assertEqual(len(velocity), len(self.test_points()))


if __name__ == "__main__":
    unittest.main()
