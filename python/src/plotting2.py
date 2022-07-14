from typing import Optional
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.offsetbox
from matplotlib.axes import Axes
import tqdm
import tqdm.notebook
from fit_types import (StrokeSolution, TrajectorySolution, LetterSolution, SlnStrokeParameters)


def expand_range(vec, padding=0.1):
    l, h = np.min(vec), np.max(vec)
    return l - (h - l) * padding, h + (h - l) * padding


def condition_axes(ax: Axes, txy, iteration_number: Optional[int] = None):
    ax.axis('equal')
    # ax.set_xlim(*expand_range(txy_gt[:, 1]))
    # ax.set_ylim(*expand_range(txy_gt[:, 2]))
    ax.set(xlim=expand_range(txy[:, 1]), ylim=expand_range(txy[:, 2]))
    ax.set_aspect('equal', adjustable='box')

    if iteration_number is not None:
        ax.text(0.05, 0.9, 'Iter {:}'.format(iteration_number), transform=ax.transAxes)
    ax.legend(loc='upper right')


def display_params(ax: Axes, text: str):
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


def plot_stroke(ax: Axes,
                sol: StrokeSolution,
                iteration_number: Optional[int] = None,
                color='k') -> None:
    if ax is None:
        _, ax = plt.subplots()
    ax.plot(sol['txy'][:, 1], sol['txy'][:, 2], color + '-', \
            label='Optimized SLN curve', linewidth=1)
    ax.plot(sol['data'][:, 1], sol['data'][:, 2], color + 'x', label='MoCap Data', markersize=3)
    condition_axes(ax, sol['data'], iteration_number)
    display_params(ax, 'params: {:}'.format(np.round(sol['params'], 2)))


def plot_trajectory(ax: Axes,
                    sol: TrajectorySolution,
                    iteration_number: Optional[int] = None,
                    disp_params: bool = True) -> None:
    """Plot a single trajectory on the axes.
    Args:
        iteration_number: The iteration number is printed on the plot for reference.
    """
    if ax is None:
        _, ax = plt.subplots()
    ax.plot(sol['data'][:, 1], sol['data'][:, 2], 'k.', label='Mocap data')
    colors = 'rgb'
    for i, (begin, end) in sol['stroke_indices'].items():
        color = colors[i % 3]
        ax.plot(sol['txy'][begin:end, 1], sol['txy'][begin:end, 2], color + '-', \
                label='Optimized SLN curve', linewidth=1)

    condition_axes(ax, sol['data'], iteration_number)

    # Display params on plot:
    if disp_params:
        if isinstance(sol['params'][0], SlnStrokeParameters):
            text = '\n'.join('params: {:}'.format(np.round(params, 2)) for params in sol['params'])
        elif isinstance(sol['params'][0],
                        tuple):  # can't do isinstance with ChebyshevStrokeParameters
            text = '\n'.join(
                'params: {:}: {:} - {:}'.format(order, np.round(xfit, 2), np.round(yfit, 2))
                for order, xfit, yfit in sol['params'])
        else:
            raise TypeError('Unknown Solution type')
        display_params(ax, text)


def plot_letter(ax: Axes, sols: LetterSolution, iteration_number: Optional[int] = None) -> None:
    """Plots all the trajectories in a letter."""
    if ax is None:
        _, ax = plt.subplots()
    keys = sols.keys()
    keys_ = list(map(lambda k: k.replace('all_', ''), keys))
    for vals in zip(*[sols[key] for key in keys]):
        plot_trajectory(ax,
                        TrajectorySolution(**{key: val for key, val in zip(keys_, vals)}),
                        iteration_number,
                        disp_params=False)
    ax.axis('equal')
    ax.get_legend().remove()
    display_params(
        ax, '\n'.join(f'params: {traji:}-{strokei:}: {np.round(params, 2):}'
                      for traji, traj in enumerate(sols['params'])
                      for strokei, params in enumerate(traj)))


# def animate_trajectories(
#         ax: Axes,
#         trajectories: Letter,
#         sols_and_histories: LetterSolutionAndHistory,
#         is_notebook: bool = False,
#         animation_oversample: int = 1,
#         save_animation_fname: Optional[str] = None) -> matplotlib.animation.Animation:
#     """Animates the optimization process for a letter."""

#     def update(i):
#         ax.cla()
#         for strokes, (_, history) in zip(trajectories, sols_and_histories):
#             plot_trajectory(ax, strokes, history[min(i, len(history) - 1)], iteration_number=i)
#         ax.get_legend().remove()
#         ax.axis('equal')
#         if i >= len(history) - 1:
#             progress_bar.close()

#     max_iterations = max(len(history) for _, history in sols_and_histories)
#     tqdm_ = tqdm.notebook if is_notebook else tqdm
#     if save_animation_fname is not None:
#         with tqdm_.trange(0, max_iterations, animation_oversample) as progress_bar:
#             progress_bar.set_description('Saving Animation')
#             ani = matplotlib.animation.FuncAnimation(ax.figure, update, frames=progress_bar)
#             ani.save(save_animation_fname, writer=matplotlib.animation.FFMpegWriter(fps=60))
#     progress_bar = tqdm_.trange(0, max_iterations, animation_oversample)
#     progress_bar.set_description('Displaying Animation')
#     return matplotlib.animation.FuncAnimation(ax.figure, update, frames=progress_bar)

# def plot_residuals(ax: Axes, strokes: Trajectory, history: History, relative=False):
#     strokes_ = np.vstack(strokes)

#     def residual(txy_from_params):
#         residuals = strokes_[:, 1:] - txy_from_params[strokes_[:, 0] == txy_from_params[:, 0], 1:]
#         # residuals = np.hstack((strokes_[:, 0].reshape(-1, 1), residuals))
#         return np.sum(np.square(residuals[:, 1:]))

#     residuals = np.array([residual(sol['txy_from_params']) for sol in history])
#     if relative:
#         ax.semilogy(np.arange(1, len(history)),
#                     residuals[1:] / residuals[:-1],
#                     'b*-',
#                     label='Relative Residuals')
#     else:
#         ax.semilogy(np.arange(len(history)), residuals, 'k*-', label='Residuals')
#     ax.set_xlabel('Iteration \#')
#     ax.set_ylabel('Residuals (SSE) (m$^2$)')
