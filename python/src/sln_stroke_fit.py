import numpy as np
import gtsam
from art_skills import SlnStrokeExpression
from gtsam.symbol_shorthand import P, X


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
        assert k * self.dt == t, f"t didn't fall on a discretization point: {t = }, {dt = }, {k = }"
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
        print(data.shape)
        for t, x, y in data:
            graph.addPriorPoint2(X(self.t2k(t)), np.array([x, y]), self.data_prior_noise_model)
        return graph

    def create_initial_values(self, graph: gtsam.Values):
        """Automatically create the initialization values for a given graph by inserting dummy
        values with the correct data type according to the symbol character.

        Args:
            graph: Factor graph that you want to solve.

        Returns:
            gtsam.Values: The initializing values
        """
        init = gtsam.Values()
        for key in graph.keyVector():
            sym = gtsam.Symbol(key)
            if sym.chr() == ord('x'):
                # init.insert(key, np.array([0, 0]))
                init.insert(key, np.zeros(2))
            elif sym.chr() == ord('p'):
                init.insert(key, np.array([-1., 1., 0., 0., 0.2, 0.1]))
            else:
                raise RuntimeError('Symbol with unknown character encountered: ', key, sym)
        return init

    @staticmethod
    def create_params(verbosityLM: str = 'SUMMARY',
                      verbosity: str = 'SILENT',
                      maxIterations: int = 100,
                      relativeErrorTol: float = 1e-5,
                      absoluteErrorTol: float = 1e-5,
                      errorTol: float = 0):
        params = gtsam.LevenbergMarquardtParams()
        params.setVerbosityLM(verbosityLM)
        params.setVerbosity(verbosity)
        params.setMaxIterations(maxIterations)
        params.setRelativeErrorTol(relativeErrorTol)
        params.setAbsoluteErrorTol(absoluteErrorTol)
        params.setErrorTol(errorTol)
        return params

    def solve(self,
              graph: gtsam.NonlinearFactorGraph,
              initial_values: gtsam.Values = None,
              params: gtsam.LevenbergMarquardtParams = None):
        if initial_values is None:
            initial_values = self.create_initial_values(graph)
        if params is None:
            params = self.create_params()
        return gtsam.LevenbergMarquardtOptimizer(graph, initial_values, params).optimize()

    def query_estimate_at(self, values, t):
        return values.atPoint2(X(self.t2k(t)))

    def query_parameters(self, values, stroke_index):
        return values.atVector(P(stroke_index))
