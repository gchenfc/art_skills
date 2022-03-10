"""
GTSAM Copyright 2010-2022, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

SLN letter regression
Author: Gerry Chen
"""

import dataclasses
from typing import Iterable, Optional
import numpy as np
import tqdm
import gtsam
from sln_stroke_fit import SlnStrokeFit, OptimizationLoggingParams
from gtsam.symbol_shorthand import X, P


@dataclasses.dataclass
class FitParams:
    noise_integration_std: float = 0.01
    noise_data_prior_std: float = 1
    dt: Optional[float] = None
    dt_oversampling: Optional[int] = 1
    max_iters: Optional[int] = 150
    params: Optional[gtsam.LevenbergMarquardtParams] = None
    initialization_strategy_params: str = 'default'  # Other possible values: 'random'
    initialization_strategy_points: str = 'from params'  # Other possible values: 'zero', 'random'


def fit_letter(
    strokes: Iterable[np.ndarray],
    fit_params: FitParams = FitParams(),
    optimization_logging_params: OptimizationLoggingParams = OptimizationLoggingParams()):
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
                          data_prior_noise_model=noise(fit_params.noise_data_prior_std))

    # Optimization Parameters
    if fit_params.params is not None:
        params = fit_params.params
    else:
        params = fitter.create_params(
            verbosityLM='SILENT',
            relativeErrorTol=0,
            absoluteErrorTol=1e-10,
            maxIterations=fit_params.max_iters if fit_params.max_iters else 100)

    # Initial values
    initial_values = gtsam.Values()
    if fit_params.initialization_strategy_params == 'default':
        for i in range(len(strokes)):
            initial_values.insert(P(i), np.array([-0.3, 1., 0., 0., 0.5, -0.5]))
            # initial_values.insert(P(i), np.array([-.3, 1., 1.57, -4.7, 0.5, -0.5]))
            # initial_values.insert(P(i),
            #                       np.array([0.0, 1., (i) * 0.75, (i + 1) * 0.75, 0.5, -0.5]))
    elif fit_params.initialization_strategy_params == ' :D ':
        for i, stroke in enumerate(strokes):
            # TODO(gerry): clean this up
            v = np.diff(stroke[:, 1:], axis=0) / np.diff(stroke[:, 0]).reshape(-1, 1)
            angles = np.arctan2(v[:, 1], v[:, 0])
            th1 = np.mean(angles[:10])
            th2 = np.mean(angles[-10:])
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
