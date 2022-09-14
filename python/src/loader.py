"""
@file loader.py
@author Gerry Chen
@author JD Florez
@brief This file contains utility functions for loading and formatting mocap data of letters.
"""

from cProfile import label
import numpy as np
import matplotlib.pyplot as plt
import scipy.signal
from fit_types import Stroke, Strokes, Trajectory, Trajectories, Letter, Segment, Segments

DT = 1. / 120


def load_letter(letter='D', artist='max'):
    fname = 'data/all_letters_{:}.npz'.format(artist)
    """Load the data for a single letter."""
    with np.load(fname, allow_pickle=True) as data:
        return data[letter]

def autosegmentation(trajectory: np.ndarray, debug: bool = False) -> Strokes:
    """Splits a real-time trajectory into its constituent strokes by computing local minima in the acceleration.
    Uses a fixed-size sliding window.
    Args:
        trajectory (np.ndarray): The trajectory as an Nx3 array with columns time, x, y.
    Returns:
        Strokes: List of (non-overlapping strokes, where each stroke is an Nx3 array.
    """
    v = np.diff(trajectory[:, 1:], axis=0)
    speed2 = np.sum(np.square(v), axis=1)
    try:
        speed2_smooth = scipy.signal.savgol_filter(speed2, 5, 1)
    except ValueError:
        print('Warning: Trajectory too short to smooth speed for local minima checking: ', speed2)
        speed2_smooth = speed2
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

def split_trajectory(trajectory: np.ndarray, debug: bool = False) -> Strokes:
    """Splits a trajectory into its constituent strokes by computing local minima in the speed.
    Args:
        trajectory (np.ndarray): The trajectory as an Nx3 array with columns time, x, y.
    Returns:
        Strokes: List of (non-overlapping strokes, where each stroke is an Nx3 array.
    """
    #print(trajectory[0:4,:])
    v = np.diff(trajectory[:, 1:], axis=0)
    #print(v[0:4,:])
    speed2 = np.sum(np.square(v), axis=1)
    try:
        speed2_smooth = scipy.signal.savgol_filter(speed2, 5, 1)
    except ValueError:
        print('Warning: Trajectory too short to smooth speed for local minima checking: ', speed2)
        speed2_smooth = speed2
    # find local minima in speeds
    splits, _ = scipy.signal.find_peaks(-speed2_smooth, width=0.02 * 120)
    splits = list(filter(lambda i: i > 0.05 * 120, splits))
    # acceleration
    acc_vector = np.diff(v[:,:], axis=0)
    x_acc = acc_vector[:,0]
    y_acc = acc_vector[:,1]
    smoothed_acc = np.diff(speed2_smooth, axis=0)
    infls = np.where(np.diff(np.sign(smoothed_acc)))[0]

    if debug:
        fig, ax = plt.subplots(4, 1, figsize=(10,15))
        ax[0].plot(speed2, '-', linewidth=1)
        ax[0].plot(splits, speed2[splits], 'r*')
        ax[0].set_ylabel('Velocity (m/s)')
        ax[1].plot(smoothed_acc, 'b-', linewidth=1)
        ax[1].plot(splits, smoothed_acc[splits], 'r*')
        ax[1].plot(infls, smoothed_acc[infls],'k*')
        ax[1].set_ylabel('Acceleration (m/s^2)')
        ax[2].plot(x_acc, 'k-', linewidth=1, label='X-acc')
        ax[2].plot(y_acc,'g-', linewidth=1, label='Y-acc')
        ax[2].plot(splits, x_acc[splits], 'r*', linewidth=1)
        ax[2].plot(splits, y_acc[splits], 'r*', linewidth=1)
        ax[2].set_ylabel('Acceleration (m/s^2)')
        ax[2].set_xlabel('Time(s)')
        ax[2].legend()
        ax[3].plot(trajectory[:, 1], trajectory[:, 2])
        ax[3].plot(trajectory[splits, 1], trajectory[splits, 2], 'r*')
        ax[3].set_ylabel('Y-Coord (m)')
        ax[3].set_xlabel('X-Coord (m)')

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


def load_segments(letter: str = 'D', index: int = 1, artist='max', debug: bool = False) -> Segments:
    """Loads the data for a letter, where a letter is composed of multiple trajectories (pen-lift)
    and each trajectory is composed of multiple strokes.
    Args:
        letter (character): which letter to load
        index (int):  Which trajectory from the letter to load.  If index is None, load all
            trajectories.
    Returns:
        Segments: List of strokes or list of trajectories, depending on index.
    """
    letter_data = load_letter(letter, artist=artist)
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
