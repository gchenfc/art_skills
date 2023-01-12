"""topp.py
Re-time trajectories using time-optimal path parameterization from the topp-ra library.
@author Gerry
"""

import numpy as np
import pandas as pd
import scipy.signal
import toppra as ta
from toppra.constraint import (DiscretizationType, JointAccelerationConstraint,
                               JointVelocityConstraint)

from . import util


class Stroke:

    def __init__(self, stroke, clean=True):
        self.stroke = util.clean_stroke(stroke) if clean else stroke
        self.t, self.x = util.stroke2tx(stroke)
        self.xdot, self.xddot = util.derivatives(self.t, self.x)
        self.path, self.toppra_instance, self.retimed_path = None, None, None

    def txva(self):
        """Returns t, x, xdot, xddot"""
        return self.t, self.x, self.xdot, self.xddot

    def spline_interp(self):
        """Interpolates the stroke using a spline."""
        self.path = spline_interp(self.t, self.x)
        return self.path

    def retime(self, vmax=1, amax=5, path=None):
        """Retimes the stroke using TOPP-RA."""
        if self.path is None:
            self.path = self.spline_interp()
        self.toppra_instance, self.retimed_path = retime(None,
                                                         None,
                                                         vmax=vmax,
                                                         amax=amax,
                                                         path=self.path)
        return self.toppra_instance, self.retimed_path

    def sample(self, N=200, dt=None):
        """Samples the original trajectory.  Provide _either_ N or dt.
        Args:
            N (int): Number of samples
            dt (float): Sample time
            retime_kwargs (dict): Keyword arguments for retime()
        Returns:
            txva: (t, x, xdot, xddot)
        """
        if self.path is None:
            self.spline_interp()
        return sample_spline(self.path, N=N, dt=dt)

    def sample_retimed(self, N=200, dt=None, **retime_kwargs):
        """Samples the retimed trajectory.  Provide _either_ N or dt.
        Args:
            N (int): Number of samples
            dt (float): Sample time
            retime_kwargs (dict): Keyword arguments for retime()
        Returns:
            txva: (t, x, xdot, xddot)
        """
        if self.toppra_instance is None or self.retimed_path is None:
            self.retime(**retime_kwargs)
        return sample_spline(self.retimed_path, N=N, dt=dt)

    def plot_xva(self, axes, original=True, retimed=True):
        """Plots the original and retimed trajectories."""
        col = 0
        if original:
            util.plot_xva(axes[:, col], *self.sample())
            axes[0, col].set_title('Original')
            col += 1
        if retimed:
            util.plot_xva(axes[:, col], *self.sample_retimed())
            axes[0, col].set_title('Retimed')

    def create_html_anim(self, fname=None):
        """Creates an HTML animation of the original and retimed trajectories.
        Usage:
            1) display(HTML(stroke.create_html_anim()))
            2) stroke.create_html_anim('filename.html')
        """
        return util.create_html_anim(self.sample(), self.sample_retimed(), fname=fname)


def spline_interp(t, x):
    """Interpolates the stroke using a spline."""
    return ta.SplineInterpolator(scipy.signal.savgol_filter(t, 15, 3), x, bc_type='clamped')


def retime(t, x, vmax=1, amax=5, path=None):
    """Retimes the stroke using TOPP-RA."""
    if path is None:
        path = spline_interp(t, x)

    # Specify Constraints
    vel_limits = np.ones((2, 1)) * vmax  # x and y
    vel_limits = np.hstack((-vel_limits, vel_limits))  # neg & pos
    acc_limits = np.ones((2, 1)) * amax  # x and y
    acc_limits = np.hstack((-acc_limits, acc_limits))  # neg & pos
    constr_vel = JointVelocityConstraint(vel_limits)
    constr_acc = JointAccelerationConstraint(acc_limits,
                                             discretization_scheme=DiscretizationType.Interpolation)

    # Setup parameterization instance
    instance = ta.algorithm.TOPPRA([constr_vel, constr_acc], path, solver_wrapper='seidel')
    retimed_path = instance.compute_trajectory(sd_start=0, sd_end=0)

    return instance, retimed_path


def sample_spline(spline: ta.SplineInterpolator, N=200, dt=None):
    if N is not None:
        ts = np.linspace(0, spline.get_duration(), N)
    elif dt is not None:
        ts = np.arange(0, spline.get_duration(), dt)
    else:
        raise ValueError('Must specify either N or dt')
    return ts, spline.eval(ts), spline.evald(ts), spline.evaldd(ts)
