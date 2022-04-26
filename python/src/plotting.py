from typing import Optional
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.offsetbox
from matplotlib.axes import Axes
import tqdm
from fit_types import Stroke, Strokes, Trajectory, Trajectories, Letter, Segment, Segments
from fit_types import (Solution, History, LetterSolutionAndHistory, LetterSolution,
                       SlnStrokeParameters, ChebyshevStrokeParameters)


def plot_trajectory(ax: Axes,
                    strokes: Strokes,
                    sol: Solution,
                    iteration_number: Optional[int] = None) -> None:
    """Plot a single trajectory on the axes.
    Args:
        iteration_number: The iteration number is printed on the plot for reference.
    """
    if ax is None:
        _, ax = plt.subplots()
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
    if isinstance(sol['params'][0], SlnStrokeParameters):
        text = '\n'.join('params: {:}'.format(np.round(params, 2)) for params in sol['params'])
    elif isinstance(sol['params'][0], tuple):  # can't do isinstance with ChebyshevStrokeParameters
        text = '\n'.join(
            'params: {:}: {:} - {:}'.format(order, np.round(xfit, 2), np.round(yfit, 2))
            for order, xfit, yfit in sol['params'])
    else:
        raise TypeError('Unknown Solution type')
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
    ax.axis('equal')
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


def plot_residuals(ax: Axes, strokes: Trajectory, history: History, relative=False):
    strokes_ = np.vstack(strokes)
    def residual(txy_from_params):
        residuals = strokes_[:, 1:] - txy_from_params[strokes_[:, 0] == txy_from_params[:, 0], 1:]
        # residuals = np.hstack((strokes_[:, 0].reshape(-1, 1), residuals))
        return np.sum(np.square(residuals[:, 1:]))
    residuals = np.array([residual(sol['txy_from_params']) for sol in history])
    if relative:
        ax.semilogy(np.arange(1, len(history)),
                    residuals[1:] / residuals[:-1],
                    'b*-',
                    label='Relative Residuals')
    else:
        ax.semilogy(np.arange(len(history)), residuals, 'k*-', label='Residuals')
    ax.set_xlabel('Iteration \#')
    ax.set_ylabel('Residuals (SSE) (m$^2$)')
