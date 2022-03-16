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
from fit_types import Strokes, Solution, History, StrokeIndices, ChebyshevStrokeParameters


def stroke_indices(self, strokes):
    stroke_indices = {}
    for strokei, stroke in enumerate(strokes):
        kstart = int((stroke[0, 0] + 1e-9) / self.dt)
        kend = int((stroke[-1, 0] + 1e-9) / self.dt) + (1 if strokei < len(strokes) - 1 else 0)
        stroke_indices[strokei] = (kstart, kend)
    # correct for if there are time indices between the stroke data points (i.e. dt < strokedt)
    k_between_strokes = int((strokes[0][1, 0] - strokes[0][0, 0] + 1e-9) / self.dt)
    for strokei in range(len(strokes) - 1):
        if (stroke_indices[strokei + 1][0] - stroke_indices[strokei][1]) < k_between_strokes:
            stroke_indices[strokei] = (stroke_indices[strokei][0], stroke_indices[strokei + 1][0])
    return stroke_indices


def fit_trajectory(strokes: Strokes,
                   p_order: int = 3) -> tuple[Solution, History, None, StrokeIndices]:
    inds = np.cumsum([stroke.shape[0] for stroke in strokes])
    stroke_indices = {i: v for i, v in enumerate(zip(np.concatenate(([0], inds[:-1])), inds))}
    arr2dict = lambda arr: {k: v for k, v in arr}
    params = []
    for stroke in strokes:
        noise = gtsam.noiseModel.Unit.Create(1)
        fit_x = gtsam.FitBasisChebyshev1Basis(arr2dict(stroke[:, [0, 1]]), noise, p_order + 1)
        fit_y = gtsam.FitBasisChebyshev1Basis(arr2dict(stroke[:, [0, 2]]), noise, p_order + 1)
        params.append((p_order, fit_x.parameters(), fit_y.parameters()))
    txy = evaluate_parameters(np.vstack(strokes)[:, 0], params, stroke_indices)
    return Solution(params=params, txy=txy, txy_from_params=txy,
                    stroke_indices=stroke_indices), None, None, stroke_indices


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
