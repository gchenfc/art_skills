"""
GTSAM Copyright 2010-2022, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Useful Utilities
Author: Gerry Chen
"""

import tqdm
import gtsam
from fit_types import OptimizationLoggingParams
import contextlib
from art_skills import ExpressionDouble, ExpressionPoint2
import time


def is_notebook():
    try:
        shell = get_ipython().__class__.__name__
        if shell == 'ZMQInteractiveShell':
            return True  # Jupyter notebook or qtconsole
        elif shell == 'TerminalInteractiveShell':
            return False  # Terminal running IPython
        else:
            return False  # Other type (?)
    except NameError:
        return False  # Probably standard Python interpreter


def tqdm_class():
    return tqdm.tqdm_notebook if is_notebook() else tqdm.tqdm

@contextlib.contextmanager
def Time(name: str):
    start = time.perf_counter()
    yield lambda : time.perf_counter() - start
    print(f'{name} took {time.perf_counter() - start:.5f} seconds')

Double_ = lambda arg: ExpressionDouble(arg)
Point2_ = lambda arg: ExpressionPoint2(arg)

def create_params(verbosityLM: str = 'SILENT',
                  verbosity: str = 'SILENT',
                  maxIterations: int = 100,
                  relativeErrorTol: float = 1e-5,
                  absoluteErrorTol: float = 1e-5,
                  errorTol: float = 0,
                  lambdaUpperBound: float = 1e9,
                  lambdaInitial: float = 1e-5):
    params = gtsam.LevenbergMarquardtParams()
    params.setVerbosityLM(verbosityLM)
    params.setVerbosity(verbosity)
    params.setMaxIterations(maxIterations)
    params.setRelativeErrorTol(relativeErrorTol)
    params.setAbsoluteErrorTol(absoluteErrorTol)
    params.setErrorTol(errorTol)
    params.setlambdaUpperBound(lambdaUpperBound)
    params.setlambdaInitial(lambdaInitial)
    return params


def ValuesFromDict(values_dict: dict):
    values = gtsam.Values()
    for key, value in values_dict.items():
        values.insert(key, value)
    return values


def solve(graph: gtsam.NonlinearFactorGraph,
          initial_values: gtsam.Values,
          params: gtsam.LevenbergMarquardtParams = None,
          logging_params: OptimizationLoggingParams = OptimizationLoggingParams()):
    if params is None:
        params = create_params()

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

    with logging_params.progress_bar_class(total=params.getMaxIterations() + 1, mininterval=0.5) \
            if logging_params.print_progress else contextlib.nullcontext() as progress:
        if logging_params:
            params.iterationHook(0, graph.error(initial_values), graph.error(initial_values))
        return optimizer.optimize(), optim_history
