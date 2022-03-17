"""
@file loader.py
@author Gerry Chen
@author JD Florez
@brief This file contains utility functions for loading and formatting mocap data of letters.
"""

import numpy as np
import matplotlib.pyplot as plt
import scipy.signal
from fit_types import Stroke, Strokes, Trajectory, Trajectories, Letter, Segment, Segments

DT = 1. / 120


def load_letter(letter='D'):
    """Load the data for a single letter."""
    with np.load('../all_letters_jules.npz', allow_pickle=True) as data:
        return data[letter]


def split_trajectory(trajectory: np.ndarray, debug: bool = False) -> Strokes:
    """Splits a trajectory into its constituent strokes by computing local minima in the speed.
    Args:
        trajectory (np.ndarray): The trajectory as an Nx3 array with columns time, x, y.
    Returns:
        Strokes: List of (non-overlapping strokes, where each stroke is an Nx3 array.
    """
    v = np.diff(trajectory[:, 1:], axis=0)
    speed2 = np.sum(np.square(v), axis=1)
    speed2_smooth = scipy.signal.savgol_filter(speed2, 5, 1)
    # find local minima in speeds
    splits, _ = scipy.signal.find_peaks(-speed2_smooth, width=0.02 * 120)
    splits = list(filter(lambda i: i > 0.05 * 120, splits))

    if debug:
        plt.subplot(121)
        # plt.plot(speed2, 'k-')
        plt.plot(speed2_smooth, '-', linewidth=1)
        plt.plot(splits, speed2[splits], 'r*')
        plt.subplot(122)
        plt.plot(trajectory[:, 1], trajectory[:, 2])
        plt.plot(trajectory[splits, 1], trajectory[splits, 2], 'r*')

    splits = [0, *splits, trajectory.shape[0] - 1]
    return [trajectory[begin:end, :] for begin, end in zip(splits[:-1], splits[1:])]


def trajectory2strokes(trajectory: np.ndarray, debug: bool = False) -> Strokes:
    """Splits a trajectory into its constituent strokes, assuming mocap data has framerate 120Hz.
    Args:
        trajectory (np.ndarray): The trajectory as an Nx2 array with columns x, y.
    Returns:
        Strokes: List of strokes.
    """
    t = np.arange(0, trajectory.shape[0] * DT, DT).reshape(-1, 1)
    return split_trajectory(np.hstack((t, trajectory)), debug=debug)


def load_segments(letter: str = 'D', index: int = 1, debug: bool = False) -> Segments:
    """Loads the data for a letter, where a letter is composed of multiple trajectories (pen-lift)
    and each trajectory is composed of multiple strokes.
    Args:
        letter (character): which letter to load
        index (int):  Which trajectory from the letter to load.  If index is None, load all
            trajectories.
    Returns:
        Segments: List of strokes or list of trajectories, depending on index.
    """
    letter_data = load_letter(letter)
    if index is None:  # return all strokes
        return [
            trajectory2strokes(letter_data[i], debug=debug) for i in range(1, len(letter_data), 2)
        ]
    return trajectory2strokes(letter_data[index], debug=debug)


if __name__ == '__main__':
    for letter in 'ABCD':
        plt.figure(figsize=(10, 5))
        load_segments(letter, index=None, debug=True)
        plt.show()
