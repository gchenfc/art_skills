"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Spline-based letter fitting
@author Gerry Chen
"""

from multiprocessing.sharedctypes import Value
from typing import Iterable
import numpy as np
import scipy.optimize
import scipy.interpolate
from fit_types import (Strokes, Letter, Solution, History, StrokeIndices, ChebyshevStrokeParameters,
                       LetterSolutionAndHistory)
import chebyshev_fitter


class SplineEvaluator:

    @staticmethod
    def evaluate_trajectory(t: np.ndarray, params: np.ndarray):
        interpolator = scipy.interpolate.interp1d(params[:, 0],
                                                  params[:, 1:],
                                                  axis=0,
                                                  kind='cubic',
                                                  bounds_error=False,
                                                  fill_value='extrapolate',
                                                  assume_sorted=False)
        return interpolator(t)

    @staticmethod
    def flatten(arr):
        return arr.T.flatten()

    @staticmethod
    def expand(params):
        return params.reshape(3, -1).T


def fit_trajectory(strokes: Strokes,
                   p_order: int = 3) -> tuple[Solution, History, None, StrokeIndices]:

    data = np.vstack(strokes)
    N_nodes = p_order * len(strokes) + 1

    def obj(params):
        return np.sum(np.square(
            SplineEvaluator.evaluate_trajectory(data[:, 0], SplineEvaluator.expand(params)) -
            data[:, 1:]),
                      axis=1)

    # initialize
    init_indices = np.fix(np.linspace(0, data.shape[0] - 1, N_nodes)).astype(int)
    # params0 = SplineEvaluator.flatten(data[init_indices, :] + np.random.rand(N_nodes, 3) * 1e-2)
    params0 = SplineEvaluator.flatten(data[init_indices, :])

    # optimize
    try:
        params_opt, ret_flag = scipy.optimize.leastsq(obj, params0, maxfev=1000 * (N_nodes * 3 + 1))
    except (ValueError, TypeError):
        print("WARNING: spline fit failed")
        params_opt = SplineEvaluator.flatten(
            np.hstack((data[:, 0:1],
                       np.repeat(np.mean(data[:, 1:], axis=0).reshape(1, 2), data.shape[0],
                                 axis=0))))
    params = SplineEvaluator.expand(params_opt)

    # return
    txy = np.hstack((data[:, 0:1], SplineEvaluator.evaluate_trajectory(data[:, 0], params)))
    stroke_indices = chebyshev_fitter.compute_stroke_indices(strokes)
    return (Solution(params=params, txy=txy, txy_from_params=txy,
                     stroke_indices=stroke_indices), None, None, stroke_indices)


def fit_letter(trajectories: Letter, p_order: int = 3) -> LetterSolutionAndHistory:
    all_sols_and_histories = []
    for strokes in trajectories:
        sol, history, _, _ = fit_trajectory(strokes, p_order=p_order)
        all_sols_and_histories.append((sol, history))
    return all_sols_and_histories


if __name__ == '__main__':
    import loader, plotting
    import matplotlib.pyplot as plt
    strokes = loader.load_segments('D', index=1)
    sol, _, _, stroke_indices = fit_trajectory(strokes, p_order=4)
    fig, ax = plt.subplots()
    plotting.plot_trajectory(ax, strokes, sol)
    plt.show()
