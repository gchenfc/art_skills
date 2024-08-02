import numpy as np
import matplotlib.pyplot as plt
import torch
from cycler import cycler

drawing_lims = dict(x=(0, 1), y=(0, 1))


def integrate_action(action, obs=None, x0=[0, 0]):
    if len(action.shape) == 1:
        # Pen-up column only
        assert obs is not None, 'Expected obs to be provided when action is pen-up only.'
        action = np.concatenate((np.full(
            (action.shape[0], 2), np.nan), action[:, None]),
                                axis=1)

    pen_up = action[:, 2] > 0.5

    if obs is None:
        obs = np.concatenate(([[0, 0]], np.cumsum(action[:, :2], axis=0))) + x0

    return obs, action, pen_up


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
    obs, action, pen_up = integrate_action(action, obs, x0)

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
              dataset.unnormalize_action(action_n)
              if dataset is not None else action_n,
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


def batchify(func, in_args=[0], out_args=[0]):

    def process_args(args, index):
        return ((arg[index] if argi in in_args else arg)
                for argi, arg in enumerate(args))

    def wrapper(*args):
        B = args[in_args[0]].shape[0]
        NARGS = len(args)
        outs_by_batch = [func(*process_args(args, b)) for b in range(B)]
        # Transpose
        if isinstance(outs_by_batch[0], tuple):
            outs = tuple([out[argi] for out in outs_by_batch]
                         for argi in range(NARGS))
            # Torch-ify out_args
            outs = tuple(
                (torch.stack(arg, axis=0) if argi in out_args else arg)
                for argi, arg in enumerate(outs))
            return outs if len(outs) > 1 else outs[0]
        else:
            return torch.stack(outs_by_batch, axis=0)

    return wrapper


def wrap_numpy_fn(func):

    def wrapped(*obs):
        out = func(*[obs_.detach().cpu().numpy() for obs_ in obs])
        if isinstance(out, tuple):
            return tuple(
                torch.tensor(out_, device=obs[0].device) for out_ in out)
        else:
            return torch.tensor(out, device=obs[0].device)

    return wrapped


def test_batchify():
    a = torch.tensor([[1, 2], [3, 4], [5, 6]])
    b = torch.tensor([[7, 8], [9, 10], [11, 12]])
    assert_ = lambda act, exp, msg: torch.testing.assert_close(
        act, exp, rtol=0, atol=0, msg=msg)

    # Test single input and single output
    func = lambda a: a + 1
    actual = batchify(func)(a)
    expected = a + 1
    assert_(actual, expected, msg="output doesn't match")

    # Test double input and double output
    func = lambda a, b: (a + b, a - b)
    actual = batchify(func, in_args=[0, 1], out_args=[0, 1])(a, b)
    expected = (a + b, a - b)
    assert_(actual[0], expected[0], msg="output 0 doesn't match")
    assert_(actual[1], expected[1], msg="output 1 doesn't match")

    # Test single input and double output
    actual = batchify(func, in_args=[0], out_args=[0, 1])(a, b[0])
    expected = (a + b[0], a - b[0])
    assert_(actual[0], expected[0], msg="output 0 doesn't match")
    assert_(actual[1], expected[1], msg="output 1 doesn't match")


def test_wrap_numpy_fn():
    assert_ = lambda act, exp, msg: torch.testing.assert_close(
        act, exp, rtol=0, atol=0, msg=msg)

    func = lambda a, b: (a + b, a - b)
    wrapped = wrap_numpy_fn(func)
    a = torch.tensor([1, 2])
    b = torch.tensor([3, 4])
    actual = wrapped(a, b)
    expected = (a + b, a - b)
    assert_(actual[0], expected[0], msg="output 0 doesn't match")
    assert_(actual[1], expected[1], msg="output 1 doesn't match")


if __name__ == '__main__':
    test_batchify()
    test_wrap_numpy_fn()
