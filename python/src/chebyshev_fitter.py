"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Chebyshev fitting
@author JD Florez-Castillo
@author Gerry Chen
"""

from typing import Iterable
import numpy as np
import gtsam
from fit_types import (Strokes, Letter, Solution, History, StrokeIndices, ChebyshevStrokeParameters,
                       LetterSolutionAndHistory)


def compute_stroke_indices(strokes):
    inds = np.cumsum([stroke.shape[0] for stroke in strokes])
    return {i: v for i, v in enumerate(zip(np.concatenate(([0], inds[:-1])), inds))}


def fit_trajectory(strokes: Strokes,
                   p_order: int = 3) -> tuple[Solution, History, None, StrokeIndices]:
    stroke_indices = compute_stroke_indices(strokes)
    arr2dict = lambda arr: {k: v for k, v in arr}
    params = []
    for stroke in strokes:
        noise = gtsam.noiseModel.Unit.Create(1)
        try:
            fit_x = gtsam.FitBasisChebyshev1Basis(arr2dict(stroke[:, [0, 1]]), noise, p_order + 1)
            fit_y = gtsam.FitBasisChebyshev1Basis(arr2dict(stroke[:, [0, 2]]), noise, p_order + 1)
            params.append((p_order, fit_x.parameters(), fit_y.parameters()))
        except RuntimeError:
            print('ERROR: fit failed')
            params.append((p_order, np.nan * np.zeros(p_order + 1), np.nan * np.zeros(p_order + 1)))
    txy = evaluate_parameters(np.vstack(strokes)[:, 0], params, stroke_indices)
    return Solution(params=params, txy=txy, txy_from_params=txy,
                    stroke_indices=stroke_indices), None, None, stroke_indices


def fit_letter(trajectories: Letter,
               p_order: int = 3) -> LetterSolutionAndHistory:
    all_sols_and_histories = []
    for strokes in trajectories:
        sol, history, _, _ = fit_trajectory(strokes, p_order=p_order)
        all_sols_and_histories.append((sol, history))
    return all_sols_and_histories


def evaluate_parameter(t: np.ndarray, params: ChebyshevStrokeParameters) -> np.ndarray:
    p_order, coeff_fitx, coeff_fity = params
    weights = gtsam.Chebyshev1Basis.WeightMatrix(p_order + 1, t)
    return np.stack((t, weights @ coeff_fitx, weights @ coeff_fity), axis=1)


def evaluate_parameters(t: np.ndarray, params: Iterable[ChebyshevStrokeParameters],
                        stroke_indices: StrokeIndices) -> np.ndarray:
    t_splits = (t[start:end] for start, end in stroke_indices.values())
    return np.vstack([evaluate_parameter(t_, param) for t_, param in zip(t_splits, params)])


if __name__ == '__main__':
    import loader, plotting
    import matplotlib.pyplot as plt
    strokes = loader.load_segments('D', index=1)
    sol, _, _, stroke_indices = fit_trajectory(strokes, p_order=4)
    fig, ax = plt.subplots()
    plotting.plot_trajectory(ax, strokes, sol)
    plt.show()
