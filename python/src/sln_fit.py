from dataclasses import dataclass
import dataclasses
from typing import Callable, Iterable, Optional, Type, Union
import itertools
import numpy as np
import gtsam
from gtsam.symbol_shorthand import P, X
from art_skills import SlnStrokeExpression2
from art_skills import ExpressionFactorPoint2, DebugExpressionFactorPoint2
import initialize_utils
import utils
from utils import Double_, Point2_
from fit_types import (OptimizationLoggingParams, StrokeSolution, TrajectorySolution,
                       LetterSolution, StrokeIndices)


@dataclass
class BaseFitParams:
    PosExpressionCb = Callable[[gtsam.Values, np.ndarray], None]
    PosExpressionCbArgs = [Type[SlnStrokeExpression2], float, float, float]
    PosExpressionCbGenerator = Callable[PosExpressionCbArgs, PosExpressionCb]

    # noise_model: gtsam.noiseModel.Base = gtsam.noiseModel.Isotropic.Sigma(2, 1e-1)
    noise_model: gtsam.noiseModel.Base = gtsam.noiseModel.Unit.Create(2)
    # noise_model_edge: Optional[gtsam.noiseModel.Base] = gtsam.noiseModel.Isotropic.Sigma(
    #     2, 1e-0)
    noise_model_edge: Optional[gtsam.noiseModel.Base] = None
    expression_debug_callback_generator: Optional[PosExpressionCbGenerator] = None
    pos_expression_eps: float = 1e-6
    lm_params: gtsam.NonlinearOptimizerParams = utils.create_params(  #absoluteErrorTol=0
        relativeErrorTol=0, absoluteErrorTol=1e-10)


@dataclass
class FitStrokeParams(BaseFitParams):
    initial_guess: Optional[Union[gtsam.Values, StrokeSolution]] = None

    def field_names():
        return [field.name for field in dataclasses.fields(FitStrokeParams)]

@dataclass
class FitTrajectoryParams(BaseFitParams):
    initial_guess: Optional[Union[gtsam.Values, TrajectorySolution]] = None
    strokewise: bool = False
    noise_model_connecting: gtsam.noiseModel.Base = gtsam.noiseModel.Constrained.All(2)

    def FitStrokeParams(self, index):
        data = {k:v for k, v in self.__dict__.items() if k in FitStrokeParams.field_names()}
        if data['initial_guess'] is None:
            return FitTrajectoryParams(**data)
        if isinstance(self.initial_guess, dict):  # TrajectorySolution
            data['initial_guess'] = TrajectoryUtils.solution2values(data['initial_guess'])
        data['initial_guess'] = utils.ValuesFromDict({
            P(index): data['initial_guess'].atVector(P(index)),
            X(index): data['initial_guess'].atPoint2(X(index)),
        })
        return FitStrokeParams(**data)

    def field_names():
        return [field.name for field in dataclasses.fields(FitTrajectoryParams)]

@dataclass
class FitLetterParams(FitTrajectoryParams):
    initial_guess: Optional[Union[Iterable[gtsam.Values], LetterSolution]] = None

    def FitTrajectoryParams(self, index):
        if self.initial_guess is None:
            return FitTrajectoryParams(**self.__dict__)
        data = {k: v for k, v in self.__dict__.items() if k in FitTrajectoryParams.field_names()}
        if isinstance(self.initial_guess, dict):  # LetterSolution
            data['initial_guess'] = {k: v[index] for k, v in self.initial_guess.items()}
        else:  # Iterable values
            data['initial_guess'] = self.initial_guess[index]
        return FitTrajectoryParams(**data)


def create_factor(t, x, y, stroke, fit_params, index):
    if fit_params.expression_debug_callback_generator is not None:
        return DebugExpressionFactorPoint2(
            fit_params.noise_model, np.array([x, y]),
            stroke.pos(Double_(t), Point2_(X(index)), fit_params.pos_expression_eps),
            fit_params.expression_debug_callback_generator(stroke, t, x, y))
    else:
        return ExpressionFactorPoint2(
            fit_params.noise_model, np.array([x, y]),
            stroke.pos(Double_(t), Point2_(X(index)), fit_params.pos_expression_eps))


class StrokeUtils:
    """A collection of useful conversion functions for StrokeSolution"""

    @staticmethod
    def extract_txy(values: gtsam.Values, stroke, index, t):
        return np.array([
            (t_, *stroke.pos(t_, x0=values.atPoint2(X(index)), values=values)) for t_ in t
        ])

    @staticmethod
    def values2solution(values: gtsam.Values, stroke, data, index):
        return StrokeSolution(params=values.atVector(P(index)),
                              txy=StrokeUtils.extract_txy(values, stroke, index, data[:, 0]),
                              data=data)

    @staticmethod
    def solution2values(solution: StrokeSolution, index):
        return utils.ValuesFromDict({
            P(index): solution['params'],
            X(index): solution['txy'][0, 1:]
        })

    @staticmethod
    def MSE(solution: StrokeSolution):  # Mean squared error
        return np.mean(np.square(solution['txy'][:, 1:] - solution['data'][:, 1:]))


class TrajectoryUtils:
    """A collection of useful conversion functions for TrajectorySolution"""

    @staticmethod
    def solution2values(solution: TrajectorySolution):
        return utils.ValuesFromDict(
            {P(i): params for i, params in enumerate(solution['params'])} +
            {X(i): solution['txy'][b, 1:] for i, (b, _) in solution['stroke_indices']})

    @staticmethod
    def values2solution(values: gtsam.Values, data: Iterable[np.ndarray], stroke_indices):

        def stroke_txy(index):
            stroke = SlnStrokeExpression2(P(index))
            return StrokeUtils.extract_txy(values, stroke, index, data[index][:, 0])

        sorted_keys = sorted(stroke_indices.keys())
        return TrajectorySolution(params=[values.atVector(P(index)) for index in sorted_keys],
                                  txy=np.vstack([stroke_txy(i) for i in sorted_keys]),
                                  data=np.vstack(data),
                                  stroke_indices=stroke_indices)

    @staticmethod
    def combine_stroke_solutions(sols: Iterable[StrokeSolution],
                                 stroke_indices: StrokeIndices,
                                 default: Iterable[StrokeSolution] = None) -> TrajectorySolution:
        """Combine multiple stroke solutions into a single trajectory solution.
        @param sols: A list of stroke solutions.
        @param default: If any of the sols are empty, replace with corresponding entry in default.
        """
        if default is not None:
            sols = [sol if sol is not None else default for sol, default in zip(sols, default)]
        return TrajectorySolution(params=[sol['params'] for sol in sols],
                                  txy=np.vstack([sol['txy'] for sol in sols]),
                                  data=np.vstack([sol['data'] for sol in sols]),
                                  stroke_indices=stroke_indices)


def fit_stroke(data: np.ndarray,
               index: int = 0,
               fit_params: FitStrokeParams = FitStrokeParams(),
               logging_params: OptimizationLoggingParams = OptimizationLoggingParams(),
               history_out: Iterable[StrokeSolution] = None) -> StrokeSolution:
    # create graph
    stroke = SlnStrokeExpression2(P(index))
    graph = gtsam.NonlinearFactorGraph()
    for txy in data:
        graph.add(create_factor(*txy, stroke, fit_params, index))
    if fit_params.noise_model_edge is not None:  # end-point weighting
        for t, x, y in data[[0, -1], :]:
            graph.add(
                ExpressionFactorPoint2(
                    fit_params.noise_model_edge, np.array([x, y]),
                    stroke.pos(Double_(t), Point2_(X(index)), fit_params.pos_expression_eps)))
    # create initial guess
    if fit_params.initial_guess is not None:
        if isinstance(fit_params.initial_guess, dict):  # StrokeSolution
            init = StrokeUtils.solution2values(fit_params.initial_guess, index)
        else:
            init = fit_params.initial_guess
    else:
        init = initialize_utils.create_init_values_3(data, index)
    # solve
    sol, history = utils.solve(graph,
                               init,
                               params=fit_params.lm_params,
                               logging_params=logging_params)

    # Re-format to correct output format
    if history_out is not None:
        history_out += [StrokeUtils.values2solution(sol_, stroke, data, index) for sol_ in history]
    return StrokeUtils.values2solution(sol, stroke, data, index)


def fit_trajectory(data: Iterable[np.ndarray],
                   fit_params: FitTrajectoryParams = FitTrajectoryParams(),
                   logging_params: OptimizationLoggingParams = OptimizationLoggingParams(),
                   history_out: Iterable[TrajectorySolution] = None) -> TrajectorySolution:
    if not fit_params.strokewise:
        return _fit_trajectory_connected(data, fit_params, logging_params, history_out)
    sols = []
    histories = []
    for i, stroke_data in enumerate(data):
        history = []
        sol = fit_stroke(stroke_data,
                         fit_params=fit_params.FitStrokeParams(i),
                         logging_params=logging_params,
                         history_out=history)
        sols.append(sol)
        histories.append(history)

    stroke_indices = [0, *np.cumsum([stroke_data.shape[0] for stroke_data in data])]
    stroke_indices = {i: bnd for i, bnd in enumerate(zip(stroke_indices[:-1], stroke_indices[1:]))}

    if history_out is not None:
        for traj_sol in itertools.zip_longest(*histories, fillvalue=None):
            history_out.append(
                TrajectoryUtils.combine_stroke_solutions(traj_sol, stroke_indices, default=sols))

    return TrajectoryUtils.combine_stroke_solutions(sols, stroke_indices)


def _fit_trajectory_connected(
        data: Iterable[np.ndarray],
        fit_params: FitTrajectoryParams = FitTrajectoryParams(),
        logging_params: OptimizationLoggingParams = OptimizationLoggingParams(),
        history_out: Iterable[TrajectorySolution] = None) -> TrajectorySolution:
    # create graph
    graph = gtsam.NonlinearFactorGraph()
    for strokei, stroke_data in enumerate(data):
        stroke = SlnStrokeExpression2(P(strokei))
        for txy in stroke_data:
            graph.add(create_factor(*txy, stroke, fit_params, strokei))
        if fit_params.noise_model_edge is not None:  # end-point weighting
            for t, x, y in stroke_data[[0, -1], :]:
                graph.add(
                    ExpressionFactorPoint2(
                        fit_params.noise_model_edge, np.array([x, y]),
                        stroke.pos(Double_(t), Point2_(X(strokei)), fit_params.pos_expression_eps)))
        if strokei < len(data) - 1:  # connectedness constraint
            stroke_end = stroke.pos(Double_(stroke_data[-1, 0]), Point2_(X(strokei)),
                                    fit_params.pos_expression_eps)
            graph.add(
                ExpressionFactorPoint2(fit_params.noise_model_connecting, np.array([0, 0]),
                                       stroke_end - Point2_(X(strokei + 1))))
    # create initial guess
    if fit_params.initial_guess is not None:
        if isinstance(fit_params.initial_guess, dict):  # StrokeSolution
            init = TrajectoryUtils.solution2values(fit_params.initial_guess)
        else:
            init = fit_params.initial_guess
    else:
        init = gtsam.Values()
        for i, stroke_data in enumerate(data):
            init.insert(initialize_utils.create_init_values_3(stroke_data, i))
    # solve
    sol, history = utils.solve(graph,
                               init,
                               params=fit_params.lm_params,
                               logging_params=logging_params)

    stroke_indices = [0, *np.cumsum([stroke_data.shape[0] for stroke_data in data])]
    stroke_indices = {i: bnd for i, bnd in enumerate(zip(stroke_indices[:-1], stroke_indices[1:]))}

    if history_out is not None:
        history_out += [TrajectoryUtils.values2solution(s, data, stroke_indices) for s in history]

    return TrajectoryUtils.values2solution(sol, data, stroke_indices)


def fit_letter(data,
               fit_params: FitLetterParams = FitLetterParams(),
               logging_params: OptimizationLoggingParams = OptimizationLoggingParams(),
               history_out: Iterable[LetterSolution] = None) -> LetterSolution:
    sols = []
    histories = []
    for i, trajectory_data in enumerate(data):
        history = []
        sol = fit_trajectory(trajectory_data,
                             fit_params=fit_params.FitTrajectoryParams(i),
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
