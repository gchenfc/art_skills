import numpy as np
import gtsam
from art_skills import SlnStrokeExpression2
from art_skills import ExpressionFactorPoint2, ExpressionDouble, ExpressionPoint2
from gtsam.symbol_shorthand import P, X
from typing import Iterable, Type
import tqdm
import dataclasses
import contextlib
from fit_types import Stroke


@dataclasses.dataclass
class OptimizationLoggingParams:
    print_progress: bool = True
    log_optimization_values: bool = False
    progress_bar_description: str = 'Fitting Stroke'
    progress_bar_class: Type[tqdm.tqdm] = tqdm.tqdm

    def __bool__(self):
        return self.print_progress or self.log_optimization_values


class SlnStrokeFit:
    """SlnStrokeFit is a class of helper functions to create a factor graph for fitting SLN strokes
       to data.
    """

    def __init__(self,
                 index: int,
                 data_prior_noise_model=gtsam.noiseModel.Isotropic.Sigma(2, 1.0),
                 reparameterize: bool = False,
                 flip_parameters_at_end: bool = True):
        self.index = index
        self.data_prior_noise_model = data_prior_noise_model
        self.reparameterize = reparameterize
        self.SlnStrokeExpression = (SlnStrokeExpression2 if not reparameterize else
                                    SlnStrokeExpression2.CreateSlnStrokeExpressionReparameterized)
        self.flip_parameters_at_end = flip_parameters_at_end

        self.stroke = self.SlnStrokeExpression(P(index))
        self.x0 = ExpressionPoint2(X(index))

    def data_prior_factors(self, data: np.ndarray) -> gtsam.NonlinearFactorGraph:
        """Returns a graph containing the priors for a data matrix.

        Args:
            data (np.ndarray): Nx3 array containing data points (t, x, y)

        Returns:
            gtsam.NonlinearFactorGraph: graph containing priors.
        """
        graph = gtsam.NonlinearFactorGraph()
        for t, x, y in data:
            factor = ExpressionFactorPoint2(self.data_prior_noise_model, np.array([x, y]),
                                            self.stroke.pos(ExpressionDouble(t), self.x0))
            graph.add(factor)
        return graph

    def create_initial_values(self, init: gtsam.Values = gtsam.Values()):
        """Automatically create the initialization values for a given graph by inserting dummy
        values with the correct data type according to the symbol character.

        Args:
            graph: Factor graph that you want to solve.

        Returns:
            gtsam.Values: The initializing values
        """
        if P(self.index) not in init.keys():
            init.insert(P(self.index), np.array([-0.1, 1., 0., 0.1, 0.2, -0.9]))
        if X(self.index) not in init.keys():
            init.insert(X(self.index), np.zeros((2, 1)))
        return init

    def reparameterize_values(self, values: gtsam.Values, inplace: bool = True):
        if not inplace:
            values = gtsam.Values(values)
        if self.reparameterize:
            for key in filter(lambda key: gtsam.Symbol(key).chr() == ord('p'), values.keys()):
                param = values.atVector(key)
                param[1] = np.log(param[1])
                param[4] = np.log(param[4])
                values.update(key, param)
        return values

    @staticmethod
    def create_params(verbosityLM: str = 'SILENT',
                      verbosity: str = 'SILENT',
                      maxIterations: int = 100,
                      relativeErrorTol: float = 1e-5,
                      absoluteErrorTol: float = 1e-5,
                      errorTol: float = 0,
                      lambdaUpperBound: float = 1e9):
        params = gtsam.LevenbergMarquardtParams()
        params.setVerbosityLM(verbosityLM)
        params.setVerbosity(verbosity)
        params.setMaxIterations(maxIterations)
        params.setRelativeErrorTol(relativeErrorTol)
        params.setAbsoluteErrorTol(absoluteErrorTol)
        params.setErrorTol(errorTol)
        params.setlambdaUpperBound(lambdaUpperBound)
        return params

    def solve(self,
              graph: gtsam.NonlinearFactorGraph,
              initial_values: gtsam.Values = gtsam.Values(),
              params: gtsam.LevenbergMarquardtParams = None,
              logging_params: OptimizationLoggingParams = OptimizationLoggingParams()):
        initial_values = self.create_initial_values(gtsam.Values(initial_values))
        if params is None:
            params = self.create_params()

        optim_history = []
        if logging_params:

            def iteration_hook(iter, error_before, error_after):
                # Python is magical and lets us use progress & optimizer before they are defined
                if logging_params.print_progress:
                    progress.update(1)
                    progress.set_description(logging_params.progress_bar_description)
                    progress.set_postfix(error='{:.2e}'.format(error_after),
                                         change_abs='{:.2e}'.format(error_before - error_after),
                                         change_rel='{:.2e}'.format(
                                             (error_before - error_after) / error_before),
                                         Lambda=optimizer.lambda_())
                if logging_params.log_optimization_values:
                    optim_history.append(gtsam.Values(optimizer.values()))

            params.iterationHook = iteration_hook

        self.reparameterize_values(initial_values, inplace=True)
        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_values, params)

        with logging_params.progress_bar_class(total=params.getMaxIterations() + 1) \
                if logging_params.print_progress else contextlib.nullcontext() as progress:
            if logging_params:
                params.iterationHook(0, graph.error(initial_values), graph.error(initial_values))
            return optimizer.optimize(), optim_history

    def fit_stroke(self,
                   stroke: Stroke,
                   initial_values: gtsam.Values = gtsam.Values(),
                   params: gtsam.LevenbergMarquardtParams = None,
                   logging_params: OptimizationLoggingParams = OptimizationLoggingParams()):
        return self.solve(self.data_prior_factors(stroke),
                          initial_values=initial_values,
                          params=params if params is not None else self.create_params(),
                          logging_params=logging_params)

    def query_estimate_at(self, values, t):
        return self.stroke.pos(t, True, values.atPoint2(X(self.index)), values)

    def query_trajectory(self, values, ts):
        return np.array([[t, *self.query_estimate_at(values, t)] for t in ts])

    def query_parameters(self, values):
        params = values.atVector(P(self.index))
        if self.reparameterize:
            params[1] = np.exp(params[1])
            params[4] = np.exp(params[4])
        if self.flip_parameters_at_end:
            D, th1, th2, sigma = params[1:5]
            flips = 0
            if D < 0:
                D = -D
                flips += 1
            if sigma < 0:
                sigma = -sigma
                flips += 1
            if flips % 2:
                th1 += np.pi
                th2 += np.pi
            params[1:5] = [D, th1, th2, sigma]
        return params

    def compute_trajectory_from_parameters(self, x0, params, ts):
        values = gtsam.Values()
        values.insert(P(self.index), params)
        return [self.stroke.pos(t, True, x0, values) for t in ts]
