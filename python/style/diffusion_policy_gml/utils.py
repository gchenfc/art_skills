import numpy as np
import matplotlib.pyplot as plt
from cycler import cycler

drawing_lims = dict(x=(0, 1), y=(0, 1))


def plot_traj(ax,
              action,
              obs=None,
              x0=[0, 0],
              clean=True,
              travel_ls='k:',
              travel_kwargs=dict(),
              line_ls='.-',
              **line_kwargs):
    """Plot the trajectory of the pen on the canvas, including travel strokes.
    Usage:
        fig, ax = plt.subplots()
        plot_traj(ax, action)
    Args:
        ax: matplotlib axis object
        action: un-normalized action sequence (dx/dy)
        obs [None]: unnormalized observation sequence (x/y) [optional, ignores action_n if provided]
        x0 [[0, 0]]: initial position
        clean [True]: whether to plot in "clean mode" (no grid, no ticks, scaled to [0, 1])
        travel_ls ['k:']: linestyle for travel strokes
        travel_kwargs [{}]: keyword arguments for travel strokes
        line_ls ['.-']: linestyle for drawing strokes
        line_kwargs [{}]: keyword arguments for drawing strokes
    """
    pen_up = action[:, 2] > 0.5

    if obs is None:
        obs = np.concatenate(([[0, 0]], np.cumsum(action[:, :2], axis=0))) + x0

    for i in np.argwhere(pen_up).flatten():
        travel = obs[i] + [[0, 0], action[i, :2].tolist()]
        ax.plot(*travel.T, travel_ls, **travel_kwargs)
    s = -1
    for i in np.argwhere(pen_up).flatten():
        ax.plot(*obs[s + 1:i + 1].T, line_ls, **line_kwargs)
        s = i
    ax.plot(*obs[s + 1:].T, line_ls, **line_kwargs)

    if clean:
        ax.grid(False)
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_xlim(*drawing_lims['x'])
        ax.set_ylim(*drawing_lims['y'])
        ax.set_aspect('equal')


def plot_result(dataset,
                action_n,
                obs,
                clean=True,
                use_obs=False,
                axes=None,
                traj_line_kwargs={},
                **fig_kwargs):
    layout = '''
    AAx
    AAd
    AAp
    '''
    if axes is None:
        fig, axes = plt.subplot_mosaic(layout, figsize=(8, 6), **fig_kwargs)
        custom_cycler = (cycler(color=['r', 'g', 'b', 'k']))
        for ax in [axes[k] for k in 'xdp']:
            ax.set_prop_cycle(custom_cycler)
    else:
        fig = axes['A'].get_figure()
    if len(action_n.shape) == 1:
        action_n = action_n[:, None]
    if action_n.shape[1] == 1:
        use_obs = True
    if use_obs:
        assert action_n.shape[1] == 1, 'Expected penup only action sequence.'
        # Add 2 columns of nan's to action_n
        action_n = np.concatenate((np.full(
            (action_n.shape[0], 2), np.nan), action_n),
                                  axis=1)
    plot_traj(axes['A'],
              dataset.unnormalize_action(action_n),
              obs=obs if use_obs else None,
              x0=obs[0],
              clean=clean,
              **traj_line_kwargs)

    axes['x'].plot(obs, alpha=0.5)
    axes['d'].plot(action_n[:, :2], alpha=0.5)
    axes['p'].plot(action_n[:, 2], alpha=0.5)
    axes['x'].set_title('Observation')
    axes['d'].set_title('Action')
    axes['p'].set_title('Pen Up')

    if clean:
        xmin, xmax = drawing_lims['x']
        ymin, ymax = drawing_lims['y']
        axes['A'].set_xticks([xmin, (xmin + xmax) / 2, xmax])
        axes['A'].set_yticks([ymin, (ymin + ymax) / 2, ymax])

    fig.tight_layout()
    return fig, axes


def kill_after_penlift(x, penlift_index):
    x = x * 1
    if len(x.shape) == 3:
        return np.stack([
            kill_after_penlift(x[i], penlift_index) for i in range(x.shape[0])
        ])

    index = np.nonzero(x[:, penlift_index] > 0.5)[0]
    if len(index) > 0:
        index = index[0]
        x[index + 1:] = np.nan

    return x
