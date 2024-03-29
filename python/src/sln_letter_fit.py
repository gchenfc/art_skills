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
from fit_types import FitParams
import loader, plotting, utils, initialize_utils


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
    initial_values = initialize_utils.create_init_values(fit_params, strokes, fitter)

    # Solve
    (sol, history), stroke_indices = fitter.fit_stroke(strokes,
                                                       initial_values=initial_values,
                                                       params=params,
                                                       logging_params=optimization_logging_params,
                                                       strokewise=fit_params.strokewise)

    # Extract and return
    def extract(values):
        params = [fitter.query_parameters(values, k) for k in range(len(strokes))]
        t = np.arange(min(stroke[0, 0] for stroke in strokes),
                      max(stroke[-1, 0] for stroke in strokes) + dt / 2, dt).reshape(-1, 1)
        txy = np.hstack((t, [fitter.query_estimate_at(values, t_) for t_ in t]))
        txy_from_params = fitter.compute_trajectory_from_parameters(
            txy, params, stroke_indices, strokewise=fit_params.strokewise)
        txy_from_params = np.hstack((t, txy_from_params))
        return {
            'params': params,
            'txy': txy,
            'txy_from_params': txy_from_params,
            'stroke_indices': stroke_indices
        }

    return extract(sol), tuple(extract(est) for est in history), fitter, stroke_indices


def fit_trajectory_2_stage(
        strokes: Strokes,
        fit_params: FitParams = FitParams(),
        optimization_logging_params: OptimizationLoggingParams = OptimizationLoggingParams(),
        stage1_iters: int = 15) -> Tuple[Solution, History, SlnStrokeFit, StrokeIndices]:
    """Fits a trajectory by first optimizing velocity-wise, then position-wise.  Args are the same
    as `fit_trajectory`.

    Args:
        stage1_iters (int, Optional): number of iterations for stage1. Defaults to 15

    Returns:
        Tuple[Solution, History, SlnStrokeFit, StrokeIndices]: _description_
    """
    # TODO(gerry): append "stage 1/2" to the optimization_logging_params prefix
    sol1, history1, _, _ = fit_trajectory(strokes,
                                          fit_params=dataclasses.replace(fit_params,
                                                                         noise_integration_std=1e2,
                                                                         max_iters=15),
                                          optimization_logging_params=optimization_logging_params)
    param_inits = gtsam.Values()
    for parami, param in enumerate(sol1['params']):
        param_inits.insert(P(parami), param)
    sol2, history2, fitter, stroke_indices = fit_trajectory(
        strokes,
        fit_params=dataclasses.replace(fit_params,
                                       max_iters=max(0, fit_params.max_iters - 15),
                                       initialization_strategy_params=param_inits),
        optimization_logging_params=optimization_logging_params)
    if optimization_logging_params.log_optimization_values:
        history2 = history1 + history2[1:]
    return sol2, history2, fitter, stroke_indices


def fit_letter(trajectories: Letter,
               max_iters: int = 100,
               log_history: bool = False,
               pbar_description_prefix: str = 'Fitting Letter',
               fit_params_kwargs: Dict[str, Any] = {},
               optimization_logging_kwargs: Dict[str, Any] = {},
               use_2_stage: bool = True,
               stage1_iters: int = 15) -> LetterSolutionAndHistory:
    # Setup fitter kwargs
    fit_params_kwargs.setdefault('initialization_strategy_params', ' :D ')
    kwargs = {
        'fit_params':
            FitParams(max_iters=max_iters, **fit_params_kwargs),
        'optimization_logging_params':
            OptimizationLoggingParams(log_optimization_values=log_history,
                                      progress_bar_class=utils.tqdm_class(),
                                      **optimization_logging_kwargs)
    }
    # Setup 2-stage vs 1-stage solvers
    if use_2_stage:
        kwargs['stage1_iters'] = stage1_iters
        fit_traj = fit_trajectory_2_stage
    else:
        fit_traj = fit_trajectory

    # Solve each Trajectory
    all_sols_and_histories = []
    for traji, strokes in enumerate(trajectories):
        kwargs['optimization_logging_params'].progress_bar_description = (
            pbar_description_prefix + ', traj {:}'.format(traji))
        sol, history, _, _ = fit_traj(strokes, **kwargs)
        all_sols_and_histories.append((sol, history))
    return all_sols_and_histories


# Convenience functions to both fit and plot at the same time
def fit_and_plot_trajectory(ax,
                            strokes: Strokes,
                            max_iters: int,
                            log_history: bool,
                            pbar_description: str,
                            use_2_stage: bool = True) -> SolutionAndHistory:
    sol, history, _, _ = fit_trajectory(strokes,
                                             fit_params=FitParams(
                                                 max_iters=max_iters,
                                                 initialization_strategy_params=' :D '),
                                             optimization_logging_params=OptimizationLoggingParams(
                                                 log_optimization_values=log_history,
                                                 progress_bar_class=utils.tqdm_class(),
                                                 progress_bar_description=pbar_description),
                                             use_2_stage=use_2_stage)
    plotting.plot_trajectory(ax, strokes, sol)
    return sol, history


def fit_and_plot_trajectories(
    ax,
    letter: str,
    artist: str = 'max',
    num_strokes=None,
    trajectory_indices=(0,),
    max_iters=100,
    log_history=False,
    animate=False,
    fit_params_kwargs: Dict[str, Any] = {},
    optimization_logging_kwargs: Dict[str, Any] = {},
    use_2_stage: bool = True,
    **animate_kwargs
) -> Union[LetterSolutionAndHistory, Tuple[LetterSolutionAndHistory,
                                           matplotlib.animation.Animation]]:
    """
    Returns:
        sols_and_histories:
            For each trajectory,
                Returns 2-tuple of sol, history
    """
    all_trajectories = loader.load_segments(letter, index=None, artist=artist)
    if trajectory_indices is not None:
        all_trajectories = [all_trajectories[i] for i in trajectory_indices]
    if num_strokes is not None:
        all_trajectories = [trajectory[:num_strokes] for trajectory in all_trajectories]

    all_sols_and_histories = fit_letter(all_trajectories,
                                        max_iters=max_iters,
                                        log_history=log_history or animate,
                                        pbar_description_prefix='Fitting Letter ' + letter,
                                        fit_params_kwargs=fit_params_kwargs,
                                        optimization_logging_kwargs=optimization_logging_kwargs,
                                        use_2_stage=use_2_stage)

    if animate:
        return all_sols_and_histories, plotting.animate_trajectories(
            ax,
            all_trajectories,
            all_sols_and_histories,
            is_notebook=utils.is_notebook(),
            **animate_kwargs)
    else:
        plotting.plot_letter(ax, all_trajectories, (sol for sol, _ in all_sols_and_histories))
        return all_sols_and_histories
