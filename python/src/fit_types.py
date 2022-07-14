"""
@file types.py
@author Gerry Chen
@author JD Florez
@brief This file contains typedefs for common data types used in letter fitting.
Terminology:
    stroke: A single "agonist-antagonist" motion.  When the speed hits a local minimum, that should
        be a new stroke.
    trajectory: A continuous collection of strokes that break when the "pen" lifts.
    letter: A collection of trajectories that makes up a letter.
    segment: Either a stroke or trajectory depending on the context.
"""

from typing import Iterable, TypedDict, Union, Tuple, Dict, Optional, Type
import numpy as np
import dataclasses
import gtsam
import tqdm

# Letter data types
Stroke = np.ndarray
Trajectory = Strokes = Iterable[Stroke]
Letter = Trajectories = Iterable[Trajectory]
Segment = Union[Stroke, Trajectory]
Segments = Union[Strokes, Trajectories]

# Fit result types
ChebyshevStrokeParameters = Tuple[int, np.ndarray, np.ndarray]
SlnStrokeParameters = np.ndarray  # 6-vector
StrokeParameters = Union[ChebyshevStrokeParameters, SlnStrokeParameters]
TrajectoryParameters = Iterable[StrokeParameters]
LetterParameters = Iterable[TrajectoryParameters]
StrokeIndices = Dict[int, Tuple[int, int]]
Solution = TypedDict(
    'Solution',
    params=Iterable[StrokeParameters],
    txy=np.ndarray,  # Nx3
    txy_from_params=np.ndarray,  # Nx3
    stroke_indices=StrokeIndices,
)
History = Iterable[Solution]
SolutionAndHistory = Tuple[Solution, History]
LetterSolution = Iterable[Solution]
LetterHistory = Iterable[History]
LetterSolutionAndHistory = Iterable[SolutionAndHistory]
# Fit result types 2
StrokeSolution = TypedDict('StrokeSolution',
                           params=StrokeParameters,
                           txy=np.ndarray,
                           data=np.ndarray)
TrajectorySolution = TypedDict('TrajectorySolution',
                               params=TrajectoryParameters,
                               txy=np.ndarray,
                               data=np.ndarray,
                               stroke_indices=StrokeIndices)
LetterSolution = TypedDict('LetterSolution',
                           params=LetterParameters,
                           txy=Iterable[np.ndarray],
                           data=Iterable[np.ndarray],
                           all_stroke_indices=Iterable[StrokeIndices])
StrokeHistory = Iterable[StrokeSolution]
TrajectoryHistory = Iterable[TrajectorySolution]
LetterHistory = Iterable[LetterSolution]


# Fit optimization parameters
@dataclasses.dataclass
class FitParams:
    noise_integration_std: float = 0.01
    noise_data_prior_std: float = 1
    reparameterize: bool = False
    flip_parameters_at_end: bool = True
    dt: Optional[float] = None
    dt_oversampling: Optional[int] = 1
    max_iters: Optional[int] = 150
    params: Optional[gtsam.LevenbergMarquardtParams] = None
    initialization_strategy_params: str = ' :D '  # Other possible values: 'default', 'random'
    initialization_strategy_points: str = 'from params'  # Other possible values: 'zero', 'random',
    # 'from params enhanced'


@dataclasses.dataclass
class OptimizationLoggingParams:
    print_progress: bool = True
    log_optimization_values: bool = False
    progress_bar_description: str = 'Fitting Stroke'
    progress_bar_class: Type[tqdm.tqdm] = tqdm.tqdm

    def __bool__(self):
        return self.print_progress or self.log_optimization_values
