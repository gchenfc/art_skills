"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Generative approach to strokes, using methodology described in:

Berio17eurographics_HandwritingTrajectories.pdf
Berio21thesis_Autograff_graffiti_writing.pdf
Github: colormotor/motor repository

Author: Juan-Diego Florez, Frank Dellaert, Sang-Won Leigh
"""

from art_skills.wSL_stroke_generator import StrokeGenerator
import numpy as np
import scipy.signal
from scipy.stats import lognorm


class TrajectoryGenerator:
    """Top-level class."""

    def __init__(self, dt, t_points, delta, Ac, delta_t, T):
        """Setup the input parameters"""
        self.dt = dt  # timestep size (sec)
        self.t_points = t_points  # 2D array of target points, including origin
        self.delta = delta  # cent. angle (rad) of the stroke's circ. arc shape
        self.Ac = Ac  # shape parameter for log curve skewedness, from [0, 1]
        self.delta_t = delta_t  # rel. t-offset wrt prev. stroke occu. & durat.
        self.T = np.ones(len(t_points)-1)*T  # period of the trajectory (sec)
        self.log_eps = 1e-15

    def trajectory_setup(self):
        """Use input values to setup all necessary derived parameters"""
        strokegen = StrokeGenerator()
        sigma = strokegen.sigma(self.Ac)
        mu = strokegen.mu(sigma, self.T)
        D, D_adj, theta, theta_0 = strokegen.D_theta(self.t_points, self.delta)
        t0, t = strokegen.t0_t(self.dt, sigma, mu, self.T, self.delta_t)
        return(sigma, mu, D, theta, theta_0, t0, t)

    def trajectory_displacements(self, t, t0, sigma, mu, D, theta, theta_0, delta):
        """Use input values to generate a stroke"""
        strokegen = StrokeGenerator()
        weight = strokegen.weight(t, t0, sigma, mu)
        displacement = strokegen.displacement(self.t_points, weight, D, theta, theta_0,
                                              delta)
        return(displacement)

    def generate_trajectory(self):
        """
        trajectory: the distribution of generated points described by the
                    collection of strokes
        """
        sigma, mu, D, theta, theta_0, t0, t = self.trajectory_setup()
        trajectory = np.zeros((2, len(t)))
        strokes = []
        for i in range(len(self.t_points[0]) - 1):
            displacement = self.trajectory_displacements(t, t0[i], sigma[i],
                                                         mu[i], D[i], theta[i], theta_0[i],
                                                         self.delta[i])
            trajectory[:, :] += displacement
            strokes.append(D[i] * self.lognormal(t, t0[i], mu[i], sigma[i]))

        return(trajectory, strokes)

    def lognormal(self, x, x0, mu, sigma):
        ''' 3 parameter lognormal'''
        x = x - x0
        x = np.maximum(x, self.log_eps)
        dist = lognorm(s=sigma, loc=0, scale=np.exp(mu))
        return dist.pdf(x)

    def extract_strokes(self):
        """
        strokes: the interpolated points of each stroke that makes up a
        trajectory
        """
        traj, strokes = self.generate_trajectory()
        maxima = scipy.signal.argrelextrema(traj[0], np.greater)
        # minima = scipy.signal.argrelextrema(traj[0], np.less)
        return(maxima)

    def velocity(self):
        """
        velocity: the lognormal velocity profile, calculated from speeds
                  between generated points
        """
        trajectory, strokes = self.generate_trajectory()
        velocity = np.sqrt(np.sum((np.gradient(trajectory, axis=1)/self.dt)**2,
                                  axis=0))
        return(velocity)


class SL_TG:
    def __init__(self, p0, dt, t0, D, theta1, theta2, theta, theta_0, sigma, mu):
        """Setup the input parameters"""
        self.p0 = p0
        self.dt = dt  # timestep size (sec)
        self.t0 = t0  # 2D array of target points, including origin
        self.D = D  # cent. angle (rad) of the stroke's circ. arc shape
        # shape parameter for log curve skewedness, from [0, 1]
        self.theta1 = theta1
        self.theta2 = theta2
        self.theta = theta
        self.theta_0 = theta_0
        self.sigma = sigma
        self.mu = mu
        self.log_eps = 1e-15

    def trajectory_setup(self):
        """Use input values to setup all necessary derived parameters"""
        strokegen = StrokeGenerator()
        delta = (self.theta1 + self. theta2)/2
        t = strokegen.time(self.dt, self.sigma, self.mu, self.t0)
        return(delta, t)

    def trajectory_displacements(self, p0, t, t0, sigma, mu, D, theta, theta_0, delta):
        """Use input values to generate a stroke"""
        strokegen = StrokeGenerator()
        weight = strokegen.weight(t, t0, sigma, mu)
        displacement = strokegen.displacement(p0, weight, D, theta, theta_0, delta)
        return(displacement)

    def generate_trajectory(self):
        """
        trajectory: the distribution of generated points described by the
                    collection of strokes
        """
        delta, t = self.trajectory_setup()
        trajectory = np.zeros((2, len(t)))
        strokes = []
        for i in range(len(self.D)):
            displacement = self.trajectory_displacements(self.p0, t, self.t0[i],
                                                         self.sigma[i],
                                                         self.mu[i], self.D[i],
                                                         self.theta[i],
                                                         self.theta_0[i],
                                                         delta)
            trajectory[:, :] += displacement
            strokes.append(self.D[i] * self.lognormal(t, self.t0[i],
                                                      self.mu[i],
                                                      self.sigma[i]))
        return(trajectory, strokes, t)

    def lognormal(self, x, x0, mu, sigma):
        ''' 3 parameter lognormal'''
        x = x - x0
        x = np.maximum(x, self.log_eps)
        dist = lognorm(s=sigma, loc=0, scale=np.exp(mu))
        return dist.pdf(x)
