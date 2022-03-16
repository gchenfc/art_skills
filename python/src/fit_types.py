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

from typing import Iterable, TypedDict, Union
import numpy as np

# Letter data types
Stroke = np.ndarray
Trajectory = Strokes = Iterable[Stroke]
Letter = Trajectories = Iterable[Trajectory]
Segment = Union[Stroke, Trajectory]
Segments = Union[Strokes, Trajectories]

# Fit result types
ChebyshevStrokeParameters = tuple[int, np.ndarray, np.ndarray]
SlnStrokeParameters = np.ndarray  # 6-vector
StrokeParameters = Union[ChebyshevStrokeParameters, SlnStrokeParameters]
StrokeIndices = dict[int, tuple[int, int]]
Solution = TypedDict(
    'Solution',
    params=Iterable[StrokeParameters],
    txy=np.ndarray,  # Nx3
    txy_from_params=np.ndarray,  # Nx3
    stroke_indices=StrokeIndices,
)
History = Iterable[Solution]
SolutionAndHistory = tuple[Solution, History]
LetterSolution = Iterable[Solution]
LetterHistory = Iterable[History]
LetterSolutionAndHistory = Iterable[SolutionAndHistory]
