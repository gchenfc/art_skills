from typing import Optional
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.offsetbox
from matplotlib.axes import Axes
import tqdm
from fit_types import Stroke, Strokes, Trajectory, Trajectories, Letter, Segment, Segments
from fit_types import Solution, History, LetterSolutionAndHistory, LetterSolution


def plot_trajectory(ax: Axes,
                    strokes: Strokes,
                    sol: Solution,
                    iteration_number: Optional[int] = None) -> None:
    """Plot a single trajectory on the axes.
    Args:
        iteration_number: The iteration number is printed on the plot for reference.
    """
    txy_gt = np.vstack(strokes)
    txy_params = sol['txy_from_params']
    ax.plot(txy_gt[:, 1], txy_gt[:, 2], 'k.', label='Mocap data')
    colors = 'rgb'
    for i, (begin, end) in enumerate(sol['stroke_indices'].values()):
        color = colors[i % 3]
        ax.plot(txy_params[begin:end, 1], txy_params[begin:end, 2], color + '-', \
                label='Optimized SLN curve', linewidth=1)
        ax.plot(sol['txy'][begin:end, 1], sol['txy'][begin:end, 2], color + '.')

    def expand_range(vec, padding=0.1):
        l, h = np.min(vec), np.max(vec)
        return l - (h - l) * padding, h + (h - l) * padding

    ax.axis('equal')
    ax.set_xlim(*expand_range(txy_gt[:, 1]))
    ax.set_ylim(*expand_range(txy_gt[:, 2]))
    # ax.set_aspect('equal', adjustable='box')
    if iteration_number is not None:
        ax.text(0.05, 0.9, 'Iter {:}'.format(iteration_number), transform=ax.transAxes)
    # Display params on plot:
    text = '\n'.join('params: {:}'.format(np.round(params, 2)) for params in sol['params'])
    text_box = matplotlib.offsetbox.AnchoredText(text,
                                                 frameon=True,
                                                 loc='lower right',
                                                 pad=0.3,
                                                 bbox_to_anchor=(1, 0),
                                                 bbox_transform=ax.transAxes,
                                                 borderpad=0,
                                                 prop=dict(size=9))
    text_box.patch.set_alpha(0.4)
    ax.add_artist(text_box)
    ax.legend(loc='upper right')


def plot_letter(ax: Axes, trajectories: Trajectories, sols: LetterSolution) -> None:
    """Plots all the trajectories in a letter."""
    for strokes, sol in zip(trajectories, sols):
        plot_trajectory(ax, strokes, sol)
    ax.get_legend().remove()


def animate_trajectories(
        ax: Axes,
        trajectories: Letter,
        sols_and_histories: LetterSolutionAndHistory,
        is_notebook: bool = False,
        animation_oversample: int = 1,
        save_animation_fname: Optional[str] = None) -> matplotlib.animation.Animation:
    """Animates the optimization process for a letter."""

    def update(i):
        ax.cla()
        for strokes, (_, history) in zip(trajectories, sols_and_histories):
            plot_trajectory(ax, strokes, history[min(i, len(history) - 1)], iteration_number=i)
        ax.get_legend().remove()
        ax.axis('equal')
        if i >= len(history) - 1:
            progress_bar.close()

    max_iterations = max(len(history) for _, history in sols_and_histories)
    tqdm_ = tqdm.notebook if is_notebook else tqdm
    if save_animation_fname is not None:
        with tqdm_.trange(0, max_iterations, animation_oversample) as progress_bar:
            progress_bar.set_description('Saving Animation')
            ani = matplotlib.animation.FuncAnimation(ax.figure, update, frames=progress_bar)
            ani.save(save_animation_fname, writer=matplotlib.animation.FFMpegWriter(fps=60))
    progress_bar = tqdm_.trange(0, max_iterations, animation_oversample)
    progress_bar.set_description('Displaying Animation')
    return matplotlib.animation.FuncAnimation(ax.figure, update, frames=progress_bar)