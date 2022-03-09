import numpy as np
import gtsam
from art_skills import SlnStrokeExpression
from gtsam.symbol_shorthand import P, X
from typing import Iterable
import tqdm
import dataclasses
import contextlib


@dataclasses.dataclass
class OptimizationLoggingParams:
    print_progress: bool = True
    log_optimization_values: bool = False
    progress_bar_description: str = 'Fitting Stroke'
    progress_bar_class: type[tqdm.tqdm] = tqdm.tqdm

    def __bool__(self):
        return self.print_progress or self.log_optimization_values


class SlnStrokeFit:
    """SlnStrokeFit is a class of helper functions to create a factor graph for fitting SLN strokes
       to data.
    """

    def __init__(self,
                 dt,
                 integration_noise_model=gtsam.noiseModel.Constrained.All(2),
                 data_prior_noise_model=gtsam.noiseModel.Isotropic.Sigma(2, 1.0)):
        self.dt = dt
        self.integration_noise_model = integration_noise_model
        self.data_prior_noise_model = data_prior_noise_model

    def t2k(self, t):
        k = np.round(t / self.dt, 0).astype(int)
        assert k * self.dt == t, \
            f"t didn't fall on a discretization point: {t = }, {self.dt = }, {k = }"
        return k

    def stroke_factors(self, stroke_index: int, k_start: int,
                       k_end: int) -> gtsam.NonlinearFactorGraph:
        """Returns a graph representing the stroke "integration" (computing the points a stroke will
        lie on using "collocation").

        Args:
            stroke_index (int): Index of the stroke this should be.
            k_start (int): Time index that the stroke should start on.
            k_end (int): Time index that the stroke should end on.

        Returns:
            gtsam.NonlinearFactorGraph: graph representing the computation constraints.
        """
        graph = gtsam.NonlinearFactorGraph()
        stroke = SlnStrokeExpression(P(stroke_index))
        for k in range(k_start, k_end):
            graph.add(stroke.pos_integration_factor(k, self.dt, self.integration_noise_model))
        return graph

    def data_prior_factors(self, data: np.ndarray) -> gtsam.NonlinearFactorGraph:
        """Returns a graph containing the priors for a data matrix.

        Args:
            data (np.ndarray): Nx3 array containing data points (t, x, y)

        Returns:
            gtsam.NonlinearFactorGraph: graph containing priors.
        """
        graph = gtsam.NonlinearFactorGraph()
        for t, x, y in data:
            graph.addPriorPoint2(X(self.t2k(t)), np.array([x, y]), self.data_prior_noise_model)
        return graph

    def create_initial_values(self,
                              graph: gtsam.NonlinearFactorGraph,
                              init: gtsam.Values = gtsam.Values()):
        """Automatically create the initialization values for a given graph by inserting dummy
        values with the correct data type according to the symbol character.

        Args:
            graph: Factor graph that you want to solve.

        Returns:
            gtsam.Values: The initializing values
        """
        for key in set(graph.keyVector()) - set(init.keys()):
            sym = gtsam.Symbol(key)
            if sym.chr() == ord('x'):
                # init.insert(key, np.array([0, 0]))
                init.insert(key, np.zeros(2))
            elif sym.chr() == ord('p'):
                init.insert(key, np.array([-0.1, 1., 0., 0., 0.2, -0.9]))
            else:
                raise RuntimeError('Symbol with unknown character encountered: ', key, sym)
        return init

    def create_initial_values_from_params(self, x0, params_values: gtsam.Values, stroke_indices):
        init = gtsam.Values(params_values)
        init.insert_or_assign(X(0), x0)
        for strokei, (kstart, kend) in stroke_indices.items():
            params = params_values.atVector(P(strokei))
            stroke = SlnStrokeExpression(params)
            for k in range(kstart, kend):
                dx = stroke.displacement((k + 1) * self.dt, self.dt)
                init.insert_or_assign(X(k + 1), init.atPoint2(X(k)) + dx)
        return init

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
        initial_values = self.create_initial_values(graph, initial_values)
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

        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_values, params)

        with logging_params.progress_bar_class(total=params.getMaxIterations() + 1) \
                if logging_params.print_progress else contextlib.nullcontext() as progress:
            if logging_params:
                params.iterationHook(0, graph.error(initial_values), graph.error(initial_values))
            return optimizer.optimize(), optim_history

    def stroke_indices(self, strokes):
        stroke_indices = {}
        for strokei, stroke in enumerate(strokes):
            kstart = int((stroke[0, 0] + 1e-9) / self.dt)
            kend = int((stroke[-1, 0] + 1e-9) / self.dt) + 1
            stroke_indices[strokei] = (kstart, kend)
        # correct for if there are time indices between the stroke data points (i.e. dt < strokedt)
        k_between_strokes = int((strokes[0][1, 0] - strokes[0][0, 0] + 1e-9) / self.dt)
        for strokei in range(len(strokes) - 1):
            if (stroke_indices[strokei + 1][0] - stroke_indices[strokei][1]) < k_between_strokes:
                stroke_indices[strokei] = (stroke_indices[strokei][0],
                                           stroke_indices[strokei + 1][0])
        return stroke_indices

    def fit_stroke(self,
                   strokes: Iterable[np.ndarray],
                   initial_values: gtsam.Values = gtsam.Values(),
                   params: gtsam.LevenbergMarquardtParams = None,
                   logging_params: OptimizationLoggingParams = OptimizationLoggingParams()):
        graph = gtsam.NonlinearFactorGraph()

        stroke_indices = self.stroke_indices(strokes)
        for strokei, stroke in enumerate(strokes):
            graph.push_back(self.stroke_factors(strokei, *stroke_indices[strokei]))
            graph.push_back(self.data_prior_factors(stroke))

        return (self.solve(graph,
                           initial_values=initial_values,
                           params=params if params is not None else self.create_params(),
                           logging_params=logging_params), stroke_indices)

    def query_estimate_at(self, values, t):
        return values.atPoint2(X(self.t2k(t)))

    def query_parameters(self, values, stroke_index):
        return values.atVector(P(stroke_index))

    def compute_trajectory_from_parameters(self, x0, params, stroke_indices):
        displacements = []
        for strokei, param in enumerate(params):
            stroke = SlnStrokeExpression(param)
            displacements += [
                stroke.displacement((k + 1) * self.dt, self.dt)
                for k in range(*stroke_indices[strokei])
            ]
        return np.vstack((np.zeros((1, 2)), np.cumsum(displacements, axis=0))) + x0
