from dataclasses import dataclass
from typing import Callable, Iterable, Optional, Type
import itertools
import numpy as np
import gtsam
from gtsam.symbol_shorthand import P, X
from art_skills import SlnStrokeExpression2
from art_skills import ExpressionFactorPoint2, DebugExpressionFactorPoint2
import initialize_utils
import utils
from utils import Double_, Point2_
from fit_types import OptimizationLoggingParams, StrokeSolution, TrajectorySolution, LetterSolution


@dataclass
class FitStrokeParams:
    PosExpressionCb = Callable[[gtsam.Values, gtsam.Values], None]
    PosExpressionCbArgs = [Type[SlnStrokeExpression2], float, float, float]
    PosExpressionCbGenerator = Callable[PosExpressionCbArgs, PosExpressionCb]

    # noise_model: gtsam.noiseModel.Base = gtsam.noiseModel.Isotropic.Sigma(2, 1e-1)
    noise_model: gtsam.noiseModel.Base = gtsam.noiseModel.Unit.Create(2)
    expression_debug_callback_generator: Optional[PosExpressionCbGenerator] = None
    pos_expression_eps: float = 1e-6
    lm_params: gtsam.NonlinearOptimizerParams = utils.create_params(absoluteErrorTol=0)


def create_factor(t, x, y, stroke, fit_params):
    if fit_params.expression_debug_callback_generator is not None:
        return DebugExpressionFactorPoint2(
            fit_params.noise_model, np.array([x, y]),
            stroke.pos(Double_(t), Point2_(X(0)), fit_params.pos_expression_eps),
            fit_params.expression_debug_callback_generator(stroke, t, x, y))
    else:
        return ExpressionFactorPoint2(
            fit_params.noise_model, np.array([x, y]),
            stroke.pos(Double_(t), Point2_(X(0)), fit_params.pos_expression_eps))


def fit_stroke(data,
               index=0,
               fit_params=FitStrokeParams(),
               logging_params=OptimizationLoggingParams(),
               history_out: Iterable[StrokeSolution] = None) -> StrokeSolution:
    stroke = SlnStrokeExpression2(P(index))
    graph = gtsam.NonlinearFactorGraph()
    for txy in data:
        graph.add(create_factor(*txy, stroke, fit_params))
    init = initialize_utils.create_init_values_3(data, index)
    sol, history = utils.solve(graph,
                               init,
                               params=fit_params.lm_params,
                               logging_params=logging_params)

    # Re-format to correct output format
    def extract_txy(values, stroke, index, t):
        return np.array([
            (t_, *stroke.pos(t_, x0=values.atPoint2(X(index)), values=values)) for t_ in t
        ])

    def values2solution(values):
        return StrokeSolution(params=values.atVector(P(index)),
                              txy=extract_txy(values, stroke, index, data[:, 0]),
                              data=data)

    if history_out is not None:
        for sol_ in history:
            history_out.append(values2solution(sol_))
    return values2solution(sol)


def fit_trajectory(data,
                   fit_params=FitStrokeParams(),
                   logging_params=OptimizationLoggingParams(),
                   history_out: Iterable[TrajectorySolution] = None) -> TrajectorySolution:
    sols = []
    histories = []
    for stroke_data in data:
        history = []
        sol = fit_stroke(stroke_data,
                         fit_params=fit_params,
                         logging_params=logging_params,
                         history_out=history)
        sols.append(sol)
        histories.append(history)

    stroke_indices = [0, *np.cumsum([stroke_data.shape[0] for stroke_data in data])]
    stroke_indices = {i: bnd for i, bnd in enumerate(zip(stroke_indices[:-1], stroke_indices[1:]))}

    def combine_sols_to_traj(sols, default=sols):
        sols = [sol if sol is not None else default for sol, default in zip(sols, default)]
        return TrajectorySolution(params=[sol['params'] for sol in sols],
                                  txy=np.vstack([sol['txy'] for sol in sols]),
                                  data=np.vstack(data),
                                  stroke_indices=stroke_indices)

    if history_out is not None:
        for traj_sol in itertools.zip_longest(*histories, fillvalue=None):
            history_out.append(combine_sols_to_traj(traj_sol))

    return combine_sols_to_traj(sols)


def fit_letter(data,
               fit_params=FitStrokeParams(),
               logging_params=OptimizationLoggingParams(),
               history_out: Iterable[LetterSolution] = None) -> LetterSolution:
    sols = []
    histories = []
    for trajectory_data in data:
        history = []
        sol = fit_trajectory(trajectory_data,
                             fit_params=fit_params,
                             logging_params=logging_params,
                             history_out=history)
        sols.append(sol)
        histories.append(history)

    def trajs2letter(trajs):
        return LetterSolution(params=[traj['params'] for traj in trajs],
                              txy=[traj['txy'] for traj in trajs],
                              data=[traj['data'] for traj in trajs],
                              all_stroke_indices=[traj['stroke_indices'] for traj in trajs])

    if history_out is not None:
        for traj_sols in itertools.zip_longest(*histories, fillvalue=None):
            traj_sols = [t if t is not None else best for t, best in zip(traj_sols, sols)]
            history_out.append(trajs2letter(traj_sols))

    return trajs2letter(sols)
