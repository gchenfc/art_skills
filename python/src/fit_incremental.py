import numpy as np
import gtsam
from gtsam.symbol_shorthand import X, P
import scipy.signal
from typing import Optional

import sln_fit
from fit_types import OptimizationLoggingParams
import utils
from utils import Point2_
from art_skills import SlnStrokeExpression2


def create_graph(data, strokes, fit_params):
    graph = gtsam.NonlinearFactorGraph()
    for t, x, y in data:
        graph.add(sln_fit.create_factor2(t, x, y, strokes, fit_params))
    return graph


class IncrementalFit:

    def initialize(self, initial_data: np.ndarray):
        raise NotImplementedError()

    def step(self, data: np.ndarray, new_stroke: bool) -> np.ndarray:
        raise NotImplementedError()

    def query(self, ts: np.ndarray):
        raise NotImplementedError()


class FixedLagFitter(IncrementalFit):

    def __init__(self, window=0.625, data_skip=1, solve_skip=1):
        self.window = window
        self.data_skip = data_skip  # TODO(gerry): use this
        self.solve_skip = solve_skip
        self.history = []
        self.previous_strokes = []
        self.snr_history = []
        self.stroke_n = 0

        self.logging_params = OptimizationLoggingParams(print_progress=False)
        # self.logging_params = OptimizationLoggingParams()
        self.fit_params = sln_fit.FitStrokeParams()
        self.fit_params.lm_params.setRelativeErrorTol(1e-5)
        self.fit_params.lm_params.setMaxIterations(3)

        self.x0 = None
        self.strokes = [SlnStrokeExpression2(P(0))]
        self.initialized = False

    def initialize(self, initial_data: np.ndarray):
        self.full_stroke = initial_data
        self.new_stroke_times = [initial_data[0, 0]]
        init = gtsam.Values()
        init.insert(X(0), initial_data[0, 1:])
        init.insert(P(0), np.array([initial_data[0, 0] - 0.05, 1., 0., 0., 0.5, -0.5]))
        self.sol, _ = utils.solve(create_graph(initial_data, self.strokes, self.fit_params), init,
                                  utils.create_params(relativeErrorTol=0, absoluteErrorTol=1e-10),
                                  self.logging_params)
        self.initialized = True

        ret = self.query(self.full_stroke[:, 0])
        self.history.append((self.full_stroke.shape[0], ret,
                             (gtsam.Values(self.sol), np.array(self.new_stroke_times),
                              [s for s in self.strokes], [s for s in self.previous_strokes])))
        return ret

    # def step(self, data: np.ndarray, new_stroke: bool):
    def step(self, data: np.ndarray, force_new_stroke: Optional[bool] = None):
        if len(data.shape) == 1:
            data = data.reshape((1, -1))
        prev_t, t = self.full_stroke[-1, 0], data[-1, 0]
        if force_new_stroke is not None:
            smooth_accel, inflection_pts, new_stroke = [], [], force_new_stroke
        else:
            smooth_accel, inflection_pts, new_stroke = self.segment_trajectory()
        print("New stroke?", new_stroke)
        # add new stroke
        if new_stroke:
            self.new_stroke_times.append(prev_t)
            print("Num strokes:", len(self.new_stroke_times))
            i = len(self.new_stroke_times) - 1
            self.strokes.append(SlnStrokeExpression2(P(i)))
            p = self.sol.atVector(P(i - 1))
            self.sol.insert(P(i), np.array([data[0, 0] - 0.05, 1., p[3], p[3], 0.5, -0.5]))
            # prev_stroke_sol = self.sol.atVector(P(i - 1))
            # prev_stroke_sol[1] *= 0.9
            # prev_stroke_sol[2] *= 0.9
            # prev_stroke_sol[3] *= 0.9
            # self.sol.update(P(i - 1), prev_stroke_sol)
        # marginalize-out x0
        if prev_t < self.new_stroke_times[0] + self.window <= t:
            print("MARGINALIZING)")
            self.x0 = self.sol.atPoint2(X(0))
            self.sol.erase(X(0))
        # marginalize-out any strokes that start before window
        # if len(self.new_stroke_times) > len(self.strokes):
        #     while self.new_stroke_times[-1 - len(self.strokes)] <= t - self.window:
        while self.new_stroke_times[-len(self.strokes)] <= t - self.window:
            print(self.new_stroke_times, len(self.strokes))
            key = P(len(self.new_stroke_times) - len(self.strokes))
            print("new key", key)
            self.previous_strokes.append(SlnStrokeExpression2(self.sol.atVector(key)))
            self.sol.erase(key)
            del self.strokes[0]

        # add new data THIS IS THE HISTORY OF THE INPUT
        self.full_stroke = np.vstack((self.full_stroke, data))

        # skip solve if not enough new data
        if self.full_stroke.shape[0] % self.solve_skip >= data.shape[0]:
            return

        # Create graph
        graph = gtsam.NonlinearFactorGraph()
        if self.x0 is None:
            for t, x, y in self.full_stroke[self.full_stroke[:, 0] > t - self.window]:
                graph.add(sln_fit.create_factor2(t, x, y, self.strokes, self.fit_params))
        else:
            for t, x, y in self.full_stroke[self.full_stroke[:, 0] > t - self.window]:
                prev = sum((stroke.pos(t) for stroke in self.previous_strokes), self.x0)
                x0 = Point2_(prev)
                graph.add(sln_fit.create_factor2(t, x, y, self.strokes, self.fit_params, x0=x0))

        # Solve
        self.sol, _ = utils.solve(graph, self.sol, self.fit_params.lm_params, self.logging_params)
        updated_stroke = self.query(self.full_stroke[:, 0])
        self.history.append((self.full_stroke.shape[0], updated_stroke,
                             (gtsam.Values(self.sol), np.array(self.new_stroke_times),
                              [s for s in self.strokes], [s for s in self.previous_strokes])))

        # Updated stroke is the list of points of the stroke of the nth iteration

        # History: [
        #           datapoint number n,
        #           np.array([time, x, y]) of n rows,
        #           [time, x, y],
        #           (parameter values p0 and x0), np.array([t0])
        #          ]

        self.snr_history.append([self.full_stroke, updated_stroke, smooth_accel, inflection_pts])

        return updated_stroke

    def query(self, ts: np.ndarray):
        x0 = self.sol.atPoint2(X(0)) if self.x0 is None else self.x0
        p = lambda t: sum(
            (stroke.pos(t, values=self.sol) for stroke in self.previous_strokes + self.strokes), x0)
        return np.hstack((ts.reshape(-1, 1), np.array([p(t) for t in ts])))

    def segment_trajectory(self):
        v = np.diff(self.full_stroke[:, 1:], axis=0) / np.diff(self.full_stroke[:, 0]).reshape(
            -1, 1)
        speed = np.sum(np.square(v), axis=1)
        acceleration = np.diff(speed, axis=0) / np.diff(self.full_stroke[1:, 0])
        acc_smooth = scipy.signal.savgol_filter(acceleration, 5, 1)
        pre_infls = np.where(np.diff(np.sign(acc_smooth)) > 0)[0]
        if np.any(pre_infls):
            print(f'\t\t\t\t{pre_infls}, {len(self.new_stroke_times)}')
            # print(f'\t\t\t\t{[int(n*60) for n in self.new_stroke_times]}')
            if len(self.new_stroke_times) < np.size(pre_infls) + 1:
                return acc_smooth, pre_infls.tolist(), True
            else:
                return acc_smooth, pre_infls.tolist(), False
        else:
            return acc_smooth, [], False