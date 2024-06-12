import numpy as np

def plot_traj(ax, action, obs=None, x0=[0, 0], clean=True, travel_ls='k:', travel_kwargs=dict(), line_ls='.-', **line_kwargs):
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
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)
        ax.set_aspect('equal')
