"""
GTSAM Copyright 2010-2022, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

SLN letter regression
Author: Gerry Chen
"""

import dataclasses
from typing import Any, Dict, Iterable, Optional, Union, Tuple
import numpy as np
import matplotlib
import tqdm
import gtsam
from sln_stroke_fit import SlnStrokeFit, OptimizationLoggingParams
from gtsam.symbol_shorthand import X, P
from fit_types import Strokes, Letter
from fit_types import Solution, History, SolutionAndHistory, LetterSolutionAndHistory, StrokeIndices
import loader, plotting


@dataclasses.dataclass
class FitParams:
    noise_integration_std: float = 0.01
    noise_data_prior_std: float = 1
    reparameterize: bool = False
    flip_parameters_at_end: bool = True
    dt: Optional[float] = None
    dt_oversampling: Optional[int] = 1
    max_iters: Optional[int] = 150
    params: Optional[gtsam.LevenbergMarquardtParams] = None
    initialization_strategy_params: str = ' :D '  # Other possible values: 'default', 'random'
    initialization_strategy_points: str = 'from params'  # Other possible values: 'zero', 'random'


def fit_trajectory(
    strokes: Strokes,
    fit_params: FitParams = FitParams(),
    optimization_logging_params: OptimizationLoggingParams = OptimizationLoggingParams()
) -> Tuple[Solution, History, SlnStrokeFit, StrokeIndices]:
    """Fits a sequence of SLN strokes to letter data.
    The letter data should be given as a list of stroke datas.

    Args:
        strokes (Iterable[np.ndarray]): A list of stroke datas, where each stroke is an Nx3 numpy
            array with columns t, x, y.  The strokes should NOT overlap (i.e. the beginning of a
            stroke should be one timestep after the end of the previous stroke).  The data should be
            uniformly timed.
        fit_params (FitParams, optional): Parameters for the fitter. Defaults to FitParams().
        optimization_logging_params (OptimizationLoggingParams, optional): Parameters for logging
            the optimization process.  Defaults to OptimizationLoggingParams().
    
    Returns:
        solution (Dict): Dictionary with keys 'params' for the parameters and 'txy' for an Nx3 array
            with columns t, x, y of the final GTSAM solution.
        history (Tuple[Dict]): If optimization_logging_params.log_optimization_values, tuple of
            Dicts (same format as `solution`) length (max_iters + 1) containing the estimate at each
            iteration of the optimization.  Otherwise, empty tuple.
        fitter (SlnStrokeFit): The SlnStrokeFit object.
        stroke_indices (Dict[int, Tuple[int, int]]): The discrete indices `k` for the start and end
            of each stroke.
    """
    if fit_params.dt is not None:
        dt = fit_params.dt
    elif fit_params.dt_oversampling is not None:
        dt = (strokes[0][1, 0] - strokes[0][0, 0]) / fit_params.dt_oversampling
    else:
        raise ValueError('Either dt or dt_oversampling must be set in FitParams')

    # Fitter object
    noise = lambda std: gtsam.noiseModel.Isotropic.Sigma(2, std)
    fitter = SlnStrokeFit(dt,
                          integration_noise_model=noise(fit_params.noise_integration_std),
                          data_prior_noise_model=noise(fit_params.noise_data_prior_std),
                          reparameterize=fit_params.reparameterize,
                          flip_parameters_at_end=fit_params.flip_parameters_at_end)

    # Optimization Parameters
    if fit_params.params is not None:
        params = fit_params.params
    else:
        params = fitter.create_params(
            verbosityLM='SILENT',
            relativeErrorTol=0,
            absoluteErrorTol=1e-10,
            maxIterations=fit_params.max_iters if fit_params.max_iters is not None else 100)

    # Initial values
    initial_values = gtsam.Values()
    if fit_params.initialization_strategy_params == 'default':
        for i in range(len(strokes)):
            initial_values.insert(P(i), np.array([-0.3, 1., 0., 0., 0.5, -0.5]))
            # initial_values.insert(P(i), np.array([-.3, 1., 1.57, -4.7, 0.5, -0.5]))
            # initial_values.insert(P(i),
            #                       np.array([0.0, 1., (i) * 0.75, (i + 1) * 0.75, 0.5, -0.5]))
    elif ' :D ' in fit_params.initialization_strategy_params:
        for i, stroke in enumerate(strokes):
            # TODO(gerry): clean this up
            v = np.diff(stroke[:, 1:], axis=0) / np.diff(stroke[:, 0]).reshape(-1, 1)
            angles = np.arctan2(v[:, 1], v[:, 0])
            th1 = np.mean(angles[:10])
            if 'old' in fit_params.initialization_strategy_params:
                th2 = np.mean(angles[-10:])
            else:
                angular_displacements = np.diff(angles)
                angular_displacements = np.arctan2(np.sin(angular_displacements),
                                                np.cos(angular_displacements))
                th2 = th1 + np.sum(angular_displacements[9:])
            sigma = 0.4
            speed = np.sqrt(np.sum(np.square(v), axis=1))
            duration = stroke[-1, 0] - stroke[0, 0]
            """
            in order to get our curve to capture ~95% of the velocity profile, we want:
                argument_of_exp = -(1/2) * (ln(t-t0) - mu)^2 / sigma^2
                2*sigma == ln(t-t0) - mu
            assume t0 = 0, then
                2*sigma == ln(duration) - mu
                mu = ln(duration) - 2*sigma
                duration = exp(2*sigma + mu)
            """
            mu = np.log(duration) - 0.9  # for sigma = 0.4

            peak_speed_i = np.argmax(speed)
            tpeak = stroke[peak_speed_i, 0]
            t0 = stroke[0, 0]
            t0alt = tpeak - np.exp(mu - sigma * sigma)

            tpeak_alt = np.exp(mu - sigma * sigma)
            predicted_peak_speed = 1 / (sigma * np.sqrt(2 * np.pi) *
                                        (tpeak_alt)) * np.exp(-0.5 * sigma * sigma)
            D = speed[peak_speed_i] / predicted_peak_speed

            # print('initial values are: ', np.array([t0, D, th1, th2, sigma, mu]))

            initial_values.insert(P(i), np.array([t0, D, th1, th2, sigma, mu]))
    else:
        raise NotImplementedError('The parameter initialization strategy is not yet implemented')

    if fit_params.initialization_strategy_points == 'from params':
        initial_values = fitter.create_initial_values_from_params(strokes[0][0, 1:], initial_values,
                                                                  fitter.stroke_indices(strokes))
    elif fit_params.initialization_strategy_points == 'zero':
        tmax = max(stroke[-1, 0] for stroke in strokes)
        for k in range(fitter.t2k(tmax) + 1):
            initial_values.insert(X(k), np.zeros(2))

    # Solve
    (sol, history), stroke_indices = fitter.fit_stroke(strokes,
                                                       initial_values=initial_values,
                                                       params=params,
                                                       logging_params=optimization_logging_params)

    # Extract and return
    def extract(values):
        params = [fitter.query_parameters(values, k) for k in range(len(strokes))]
        t = np.arange(min(stroke[0, 0] for stroke in strokes),
                      max(stroke[-1, 0] for stroke in strokes) + dt / 2, dt).reshape(-1, 1)
        txy = np.hstack((t, [fitter.query_estimate_at(values, t_) for t_ in t]))
        txy_from_params = fitter.compute_trajectory_from_parameters(txy[0, 1:], params,
                                                                    stroke_indices)
        txy_from_params = np.hstack((t, txy_from_params))
        return {
            'params': params,
            'txy': txy,
            'txy_from_params': txy_from_params,
            'stroke_indices': stroke_indices
        }

    return extract(sol), tuple(extract(est) for est in history), fitter, stroke_indices


def fit_letter(trajectories: Letter,
               max_iters: int = 100,
               log_history: bool = False,
               pbar_description_prefix: str = 'Fitting Letter',
               fit_params_kwargs: Dict[str, Any] = {},
               optimization_logging_kwargs: Dict[str, Any] = {}) -> LetterSolutionAndHistory:
    fit_params_kwargs.setdefault('initialization_strategy_params', ' :D ')
    all_sols_and_histories = []
    for traji, strokes in enumerate(trajectories):
        sol, history, _, _ = fit_trajectory(
            strokes,
            fit_params=FitParams(max_iters=max_iters, **fit_params_kwargs),
            optimization_logging_params=OptimizationLoggingParams(
                log_optimization_values=log_history,
                progress_bar_class=tqdm.tqdm_notebook,
                progress_bar_description=pbar_description_prefix + ', traj {:}'.format(traji),
                **optimization_logging_kwargs),
        )
        all_sols_and_histories.append((sol, history))
    return all_sols_and_histories


# Convenience functions to both fit and plot at the same time
def fit_and_plot_trajectory(ax, strokes: Strokes, max_iters: int, log_history: bool,
                            pbar_description: str) -> SolutionAndHistory:
    sol, history, fitter, _ = fit_trajectory(strokes,
                                             fit_params=FitParams(
                                                 max_iters=max_iters,
                                                 initialization_strategy_params=' :D '),
                                             optimization_logging_params=OptimizationLoggingParams(
                                                 log_optimization_values=log_history,
                                                 progress_bar_class=tqdm.tqdm_notebook,
                                                 progress_bar_description=pbar_description))
    plotting.plot_trajectory(ax, strokes, sol)
    return sol, history


def fit_and_plot_trajectories(
    ax,
    letter: str,
    num_strokes=None,
    trajectory_indices=(0,),
    max_iters=100,
    log_history=False,
    animate=False,
    fit_params_kwargs: Dict[str, Any] = {},
    optimization_logging_kwargs: Dict[str, Any] = {},
    **animate_kwargs
) -> Union[LetterSolutionAndHistory, Tuple[LetterSolutionAndHistory,
                                           matplotlib.animation.Animation]]:
    """
    Returns:
        sols_and_histories:
            For each trajectory,
                Returns 2-tuple of sol, history
    """
    all_trajectories = loader.load_segments(letter, index=None)
    if trajectory_indices is not None:
        all_trajectories = [all_trajectories[i] for i in trajectory_indices]
    if num_strokes is not None:
        all_trajectories = [trajectory[:num_strokes] for trajectory in all_trajectories]

    all_sols_and_histories = fit_letter(all_trajectories,
                                        max_iters=max_iters,
                                        log_history=log_history or animate,
                                        pbar_description_prefix='Fitting Letter ' + letter,
                                        fit_params_kwargs=fit_params_kwargs,
                                        optimization_logging_kwargs=optimization_logging_kwargs)

    if animate:
        return all_sols_and_histories, plotting.animate_trajectories(ax,
                                                                     all_trajectories,
                                                                     all_sols_and_histories,
                                                                     is_notebook=True,
                                                                     **animate_kwargs)
    else:
        plotting.plot_letter(ax, all_trajectories, (sol for sol, _ in all_sols_and_histories))
        return all_sols_and_histories
