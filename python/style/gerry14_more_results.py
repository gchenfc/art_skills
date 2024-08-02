# %%
import torch
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
import tqdm.auto as tqdm
import functools
import hashlib
import argparse
import style.diffusion_policy_gml.utils as utils
from gerry10_edit import EditorCnn
from gerry11_all_networks import (Base, Decoupled, Finetuned, ClassifierFree1,
                                  Guidance, Control)
from gerry11_all_networks import DEVICE, HDict
from scipy.interpolate import CubicSpline
from collections import defaultdict
from load_gml import Drawing

# %% Set up editing model
model = ClassifierFree1()
editor = EditorCnn(model)
results_folder = Path('results') / 'gerry14_more_results'
results_folder.mkdir(exist_ok=True)
DT = 0.02

# %% [markdown]
# What are the results/figures we want to generate?
# * Max & Jules
# * Compute similarity & dynamics evaluation numbers for all
# * Show different random seeds
# * Show different W weight matrices (weight different parts of the trajectory)


# %% [markdown]
# # Utilities
# Cached Diffusion Edit
class HashableTensorList:

    def __init__(self, tensors):
        self.tensors = tensors

    @staticmethod
    def hash_tensor(tensor):
        np_array = tensor.detach().cpu().numpy()
        return int(hashlib.sha256(np_array.tobytes()).hexdigest(), 16)

    def __hash__(self):
        return hash(
            tuple(
                HashableTensorList.hash_tensor(tensor)
                for tensor in self.tensors))


@functools.lru_cache(maxsize=1024)
def edit_(strokes, kwargs):
    return editor.edit(strokes.tensors, **kwargs)


# lru cache isn't working idk why so just give up
# edit = lambda strokes, **kwargs: edit_(HashableTensorList(strokes),
#                                        HDict(**kwargs))
edit = lambda strokes, **kwargs: editor.edit(strokes, **kwargs)


# Slow down via interpolation
def slow_down(strokes, factor):
    if isinstance(strokes, list):
        return [slow_down(stroke, factor) for stroke in strokes]

    stroke = strokes
    t = np.linspace(0, 1, len(stroke))
    t_new = np.linspace(0, 1, max(2, int(len(stroke) * factor)))
    cs = CubicSpline(t, stroke.cpu().numpy(), axis=0)
    return torch.from_numpy(cs(t_new)).to(stroke)


def max_speed(strokes):
    if isinstance(strokes, list):
        return [max_speed(stroke) for stroke in strokes]

    return torch.max(strokes.diff(dim=0) / DT).item()


# Edit, integrate, and realign shortcut
def edit_strokes_individually(strokes,
                              max_speed_=0.3,
                              realign_with_orig=False,
                              t_start=3,
                              repeat=20,
                              global_cond_weight=1.3,
                              print_progress=False,
                              unintegrated_obs=False,
                              **kwargs):
    acts, obss = [], []
    for stroke in strokes:
        if stroke.shape[0] <= 1:
            acts.append(np.zeros((stroke.shape[0], 3)))
            obss.append(np.zeros((stroke.shape[0], 2)))
            continue
        try:
            stroke = slow_down([stroke],
                               max_speed([stroke])[0] / max_speed_)[0]
            obs, act = edit([stroke],
                            t_start=t_start,
                            repeat=repeat,
                            global_cond_weight=global_cond_weight,
                            print_progress=print_progress,
                            **kwargs)
        except ValueError as e:
            raise e
            obs = stroke[None, ...]
            act = torch.zeros((1, stroke.shape[0], 3))
        obs, act = obs.cpu().numpy()[0], act.cpu().numpy()[0]
        obs2, _, _ = utils.integrate_action(act, None, obs[0])
        if realign_with_orig:
            obs2 += stroke.mean(dim=0).cpu().numpy() - obs2.mean(axis=0)
        acts.append(act)
        if unintegrated_obs:
            obss.append(obs)
        else:
            obss.append(obs2)
    return obss, acts


# plotting utilities
def plot_strokes(ax, strokes, **kwargs):
    if isinstance(strokes[0], np.ndarray):
        strokes = map(torch.from_numpy, strokes)

    for stroke in strokes:
        utils.plot_traj(ax,
                        np.zeros((10, 3)),
                        obs=stroke.cpu().numpy(),
                        **kwargs)


colororder = plt.rcParams['axes.prop_cycle'].by_key()['color']


def render(ax, strokes, color_index=None, infill=False, **kwargs):
    if isinstance(strokes[0], torch.Tensor):
        strokes = [stroke.cpu().numpy() for stroke in strokes]

    color = colororder[color_index] if isinstance(color_index,
                                                  int) else color_index

    if infill:
        for stroke in strokes:
            ax.fill(*stroke.T, color=color, zorder=2)

        for stroke in strokes:
            stroke = np.concatenate([stroke, [stroke[0]]], axis=0)
            ax.plot(*stroke.T, 'k-', linewidth=10, solid_capstyle='round')
    else:
        for stroke in strokes:
            ax.plot(*stroke.T,
                    '-',
                    linewidth=10,
                    solid_capstyle='round',
                    color=color,
                    **kwargs)


def scale_and_shift(strokes, scale=1, shift=[0, 0]):
    if isinstance(strokes, list):
        return [
            scale_and_shift(stroke, scale=scale, shift=shift)
            for stroke in strokes
        ]
    if isinstance(strokes, torch.Tensor):
        return strokes * scale + torch.Tensor(shift).to(strokes)[None, :]
    if isinstance(strokes, np.ndarray):
        return strokes * scale + shift
    raise ValueError('strokes must be a list, torch.Tensor, or np.ndarray')


def format_axes(ax):
    ax.grid(False)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_aspect('equal')
    ax.axis('equal')


# Random other utils
def preprocess_gml(file: Path) -> list[torch.Tensor]:
    drawing = Drawing(f)
    strokes = [stroke[:, 1:3] for stroke in drawing.strokes]
    return [
        torch.from_numpy(stroke).to(DEVICE, dtype=torch.float32)
        for stroke in strokes
    ]


# %%
def max_and_jules():
    pass

    # %% [markdown]
    # # Max & Jules
    jules_path = Path('data/all_letters_jules.npz')
    max_path = Path('data/all_letters_max.npz')
    with np.load(jules_path, allow_pickle=True) as data:
        print(list(data.keys()))
        jules_data = {k: v for k, v in data.items()}
    with np.load(max_path, allow_pickle=True) as data:
        print(list(data.keys()))
        max_data = {k: v for k, v in data.items()}
    TO_KEEP_JULES = dict(A=[1, 3],
                         B=[0, 1],
                         C=[1],
                         D=[1, 3],
                         E=[1],
                         J=[1],
                         U=[1],
                         L=[1],
                         S=[1])
    TO_KEEP_MAX = dict(A=[1, 3],
                       B=[1, 3, 5],
                       C=[1],
                       D=[1, 3],
                       E=[1],
                       M=[1],
                       X=[1])
    filter_strokes = lambda strokes, inds: [strokes[i] for i in inds]
    jules_data = {
        k: filter_strokes(v, TO_KEEP_JULES[k]) if k in TO_KEEP_JULES else v
        for k, v in jules_data.items()
    }
    max_data = {
        k: filter_strokes(v, TO_KEEP_MAX[k]) if k in TO_KEEP_MAX else v
        for k, v in max_data.items()
    }

    # %% Normalize
    def normalize(data):
        all_points = np.concatenate(
            [np.concatenate(strokes, axis=0) for strokes in data.values()],
            axis=0)
        center = all_points.mean(axis=0)
        scale = np.max(all_points.std()) * 2
        return {
            k: [
                torch.from_numpy((stroke - center) / scale + 0.5).to(
                    DEVICE, torch.float32) for stroke in strokes
            ]
            for k, strokes in data.items()
        }

    jules_data_n = normalize(jules_data)
    max_data_n = normalize(max_data)
    data_n = dict(max=max_data_n, jules=jules_data_n)

    def create_word_helper(strokess, spacing=0.5):
        n = len(strokess)
        width = ((n - 1) * spacing + 1)
        place_stroke = lambda strokes, s, x, y: [(
            (stroke - 0.5) * s + torch.Tensor([x, y]).to(stroke))
                                                 for stroke in strokes]
        return sum((place_stroke(strokes, 1 / width,
                                 (0.5 + i * spacing) / width, 0.5)
                    for i, strokes in enumerate(strokess)), [])

    def create_word(string, artist, spacing=0.5):
        return create_word_helper(
            [data_n[artist][letter] for letter in string], spacing)

    # %% Plot
    fig, axes = plt.subplots(1, 2, figsize=(10, 5))
    for strokes in jules_data_n.values():
        plot_strokes(axes[0], strokes)

    for strokes in max_data_n.values():
        plot_strokes(axes[1], strokes)

    fig, axes = plt.subplots(1, 2, figsize=(10, 5))
    plot_strokes(axes[0], create_word('ABCDE', 'jules', spacing=0.5))
    plot_strokes(axes[1], create_word('ABCDE', 'max', spacing=0.5))

    # %%
    strokes = slow_down(jules_data_n['A'], 3)
    strokes = [stroke * 1 for stroke in strokes]
    print(max_speed(strokes))
    plot_strokes(plt.gca(), strokes, line_ls='k-')

    torch.manual_seed(8675309)
    for stroke in strokes:
        obs, act = edit([stroke], t_start=3, repeat=20, global_cond_weight=1.3)
        obs, act = obs.cpu().numpy()[0], act.cpu().numpy()[0]
        print(obs.shape)
        utils.plot_traj(plt.gca(),
                        act,
                        x0=obs[0],
                        linewidth=10,
                        solid_capstyle='round')

    # %% Diffusion edit batch
    def edit_and_draw2(ax1, ax2, strokess, spacing=0.5, max_speed_=0.3):
        for i, strokes in enumerate(tqdm.tqdm(strokess)):
            plot_strokes(ax1,
                         scale_and_shift(strokes, shift=[i * spacing, 0]),
                         line_ls='k:')

        for i, strokes in enumerate(tqdm.tqdm(strokess)):
            obss, actss = edit_strokes_individually(strokes,
                                                    max_speed_=0.3,
                                                    realign_with_orig=False)
            strokes_out = scale_and_shift(obss, shift=[i * spacing, 0])
            plot_strokes(ax1, strokes_out, line_ls='b-')

            render(ax2, strokes_out, color_index=i, infill=True)
            format_axes(ax1)
            format_axes(ax2)

    fig, axes = plt.subplots(1, 2, figsize=(18, 5))
    fig2, axes2 = plt.subplots(1, 2, figsize=(18, 5))

    torch.manual_seed(8675309)
    edit_and_draw2(axes[0], axes2[0],
                   [data_n['jules'][letter] for letter in 'JULES'])
    torch.manual_seed(8675309 + 2)
    edit_and_draw2(axes[1], axes2[1],
                   [data_n['max'][letter] for letter in 'MAX'])

    axes[1].lines[0].set_label('Mocap Input')
    axes[1].lines[-1].set_label('Sketch Retargeting Result')
    axes[1].legend(loc='lower right', fontsize=18)

    fig.suptitle('Sketch Retargeting Applied to Motion Capture Data',
                 fontsize=36)
    fig.tight_layout()

    fig2.suptitle(
        'Sketch Retargeting Result Example Rendering (from Mocap Input)',
        fontsize=36)
    fig2.tight_layout()

    # fig.savefig(results_folder / 'jules_max_compare.svg')
    # fig.savefig(results_folder / 'jules_max_compare.eps')
    # fig2.savefig(results_folder / 'jules_max_render.svg')
    # fig2.savefig(results_folder / 'jules_max_render.eps')


# %%
# # Compute similarity & dynamics evaluation numbers for all

pass


# %%
# # Show different random seeds
def run_for_random_seeds():
    pass

    # %%
    logs_folder = Path('data/log_gmls')
    for f in sorted(logs_folder.glob('*.json')):
        print(f.name)

    # %%
    f = logs_folder / 'danny_2024-07-31-221434.json'
    strokes = preprocess_gml(f)

    # for i, strokes in enumerate(tqdm.tqdm(strokess)):
    plot_strokes(plt.gca(), strokes, line_ls='k.-')

    fig, axes = plt.subplots(1, 2, figsize=(10, 5))
    render(axes[0], strokes, color_index='k')
    torch.manual_seed(8675309)
    obss, actss = edit_strokes_individually(strokes,
                                            max_speed_=0.3,
                                            realign_with_orig=False,
                                            repeat=20)
    render(axes[1], obss, color_index='b')

    for ax in axes.flatten():
        format_axes(ax)

    # %%
    name = f.stem.split('_')[0].title()
    realign = True

    def random_seeds(strokes):
        fig, axes = plt.subplots(3, 4, figsize=(20, 15))
        # Plot orig
        render(axes[1][0], strokes, color_index='k')
        axes[0][0].axis('off')
        axes[2][0].axis('off')

        # Plot permutations
        torch.manual_seed(8675309)
        for ax in tqdm.tqdm(axes[:, 1:].flatten()):
            obss, actss = edit_strokes_individually(strokes,
                                                    max_speed_=0.3,
                                                    realign_with_orig=realign,
                                                    repeat=20)
            strokes_out = obss
            render(ax, strokes_out, color_index='b')

        for ax in axes.flatten():
            format_axes(ax)

        fig.suptitle('Sketch Retargeting with Different Random Seeds',
                     fontsize=36,
                     x=0.625,
                     va='center')

        fig.tight_layout()
        axes[1][0].set_title('Human Input', fontsize=36, pad=20)
        # axes[0][0].text(0.5,
        #                 0,
        #                 'Human Input',
        #                 ha='center',
        #                 va='top',
        #                 fontsize=36,
        #                 transform=axes[0][0].transAxes)

        return fig, axes

    print('running on random seeds')
    fig, axes = random_seeds(strokes)
    # fig.savefig(results_folder / f'random_seeds_{name}_{realign}.svg')
    # fig.savefig(results_folder / f'random_seeds_{name}_{realign}.eps')


# %%
# # Run on all log files
def run_for_all_log_files():
    pass
    # %%
    logs_folder = Path('data/log_gmls')
    out_folder = results_folder / 'all_logs'
    out_folder.mkdir(exist_ok=True)
    realign = True

    def orig_and_retargeted(file, strokes):
        fig, axes = plt.subplots(1, 2, figsize=(10, 5))
        # Plot orig
        render(axes[0], strokes, color_index='k')
        axes[0].set_title('Human Input', fontsize=36, pad=20)

        # Plot retargeted
        torch.manual_seed(8675309)
        obss, actss = edit_strokes_individually(strokes,
                                                max_speed_=0.3,
                                                realign_with_orig=realign,
                                                repeat=20)
        render(axes[1], obss, color_index='b')

        for ax in axes.flatten():
            format_axes(ax)

        fig.suptitle(f'Sketch Retargeting ({file.stem})',
                     fontsize=36,
                     x=0.625,
                     va='center')
        fig.tight_layout()

        fig.savefig(out_folder / f'{file.stem}.svg')

    for f in tqdm.tqdm(sorted(logs_folder.glob('*.json'))):
        if (out_folder / f'{f.stem}.svg').exists():
            continue
        strokes = preprocess_gml(f)
        orig_and_retargeted(f, strokes)

    # %%
    template = (out_folder / '_gallery_template.html').read_text()

    div = lambda file: f'<div class="svg-item">{file.stem}<br /><img src="{file.name}" alt="{file.stem}"></div>'
    s = '\n'.join([div(file) for file in sorted(out_folder.glob('*.svg'))])
    with open(out_folder / '_gallery.html', 'w') as f:
        f.write(template.replace('#####SVGs#####', s))


# %% [markdown]
# # Show different W weight matrices (weight different parts of the trajectory)
def run_weight_matrices():
    pass

    # %%
    from matplotlib.colors import LinearSegmentedColormap

    cmap = plt.cm.Greys
    colors = cmap(np.arange(cmap.N))
    colors[:, -1] = 1 - colors[:, 0]
    colors[:, :3] = 0
    custom_cmap = LinearSegmentedColormap.from_list('CustomGreys', colors)

    # %%
    logs_folder = Path('data/log_gmls')
    out_folder = results_folder
    f = logs_folder / 'gerry_2024-08-02-101232.json'
    strokes = preprocess_gml(f)

    fig, axes = plt.subplots(2, 3, figsize=(15, 10), sharey=True)
    axes = axes.flatten()
    # Plot orig
    render(axes[0], strokes, color_index='k')
    axes[0].set_title('Human Input', fontsize=24, pad=10)
    # axes = axes[:3] + axes[4:]

    # First retarget, just to get the new re-timings
    obss, actss = edit_strokes_individually(strokes,
                                            max_speed_=0.3,
                                            realign_with_orig=True,
                                            t_start=0,
                                            repeat=0)
    # render(axes[1], obss, color_index='b')
    # axes[1].plot(*obss[0].T, 'k:', linewidth=1)

    # Plot retargeted
    torch.manual_seed(8675309)
    # N = strokes[0].shape[0]
    # print(N)
    # W = np.zeros((N))
    # W[0] = 1
    # W[20] = 1
    # W[37] = 1
    # W[55] = 1
    # W[70] = 1
    # W[-1] = 1
    # axes[1].plot(*strokes[0].cpu().numpy()[W == 1].T, 'ro')
    N = obss[0].shape[0] - 1

    def W1():
        W = np.zeros((N))
        W[0] = 3
        W[200] = 1
        W[370] = 1
        W[550] = 1
        W[700] = 1
        W[-1] = 7
        # smooth it out a little bit
        for _ in range(20):
            W = np.convolve(W, np.ones(10) / 10, mode='same')
        W = np.roll(W, -10)
        return W

    def W2():
        speed = np.linalg.norm(np.diff(obss[0], axis=0), axis=1) / DT
        W = 0.05 / np.clip(speed, 0.01, 1)
        return W

    def W3():
        return np.ones((N))

    def W4():
        return 1 / W2()

    def W5():
        inds = [100, 285, 460, 625, 778]
        W = np.zeros((N))
        W[inds] = 1
        for _ in range(20):
            W = np.convolve(W, np.ones(10) / 10, mode='same')
        W = np.roll(W, 5)
        return W

    Ws = [W4(), W5(), W3(), W2(), W1()]
    Ws = [W * N / np.sum(W) for W in Ws]

    obss_ = obss * 1

    vmax = max(*[np.max(W) for W in Ws])

    for ax, W in zip(axes[1:], Ws):
        print(W.shape, obss[0].shape)
        sc = ax.scatter(*obss_[0][:-1].T,
                        c=W,
                        cmap=custom_cmap,
                        vmin=0,
                        vmax=vmax,
                        s=300)

        obss, actss = edit_strokes_individually(
            strokes,
            max_speed_=0.3,
            realign_with_orig=True,
            t_start=3,
            repeat=20,
            unintegrated_obs=True,
            guidance_loss_kwargs=dict(
                W=torch.from_numpy(W).to(strokes[0])[None, :, None],
                #     # eta=100, eta_delta=0.25, eta_penup=0
                eta=100,
                eta_delta=0.25,
                eta_penup=0))

        # Terrible hack - it's not picking up enough from the position guidance so
        # I'm using the raw position signals.  But they are noisy so I'm smoothing
        # them.
        from scipy.signal import savgol_filter as sgolay
        x = sgolay(obss[0], 75, 1, axis=0)
        obss[0] = x

        render(ax, obss, color_index='g', alpha=0.75)

    axes[3].lines[-1].set_label('Retargeted Result')
    # axes[3].lines[0].set_label('Guidance')
    axes[3].plot([-1], [-1],
                 'ko',
                 markersize=10,
                 linewidth=10,
                 label='Similarity Guidance')
    # sc.legend_elements(num=1)[0][0].set_label('Guidance')
    # sc.legend_elements(num=1)
    axes[3].legend(loc='lower left', fontsize=18)

    fig.subplots_adjust(right=0.9,
                        left=0.05,
                        wspace=0.05,
                        hspace=0.05,
                        bottom=0.05)

    bottom = axes[3].get_position().y0
    top = axes[2].get_position().y1
    cbar_ax = fig.add_axes([0.92, bottom, 0.02, top - bottom])
    fig.colorbar(sc, cax=cbar_ax).set_label(
        label='Similarity Guidance Weight, $W$', size=24)

    for ax in axes.flatten():
        format_axes(ax)
        ax.set_xlim(0.1, 0.9)
        ax.set_ylim(0.1, 0.9)

    fig.suptitle(
        f'Sketch Retargeting with Various Guidance Weight Distributions',
        fontsize=36,
        # x=0.625,
        # va='center'
    )

    # fig.savefig(out_folder / f'weight_variation.eps')
    # fig.savefig(out_folder / f'weight_variation.svg')


# %% [markdown]
# # main
def main():
    parser = argparse.ArgumentParser(description='Convert logs to GML')
    parser.add_argument('--infolder',
                        type=Path,
                        default=DEFAULT_INFOLDER,
                        help=f'Default: {DEFAULT_INFOLDER}')
    parser.add_argument('--outfolder',
                        type=Path,
                        default=DEFAULT_OUTFOLDER,
                        help=f'Default: {DEFAULT_OUTFOLDER}')
    parser.add_argument('-m', '--max_and_jules', action='store_true')
    parser.add_argument('-q',
                        '--compute_similarity_dynamics',
                        action='store_true')
    parser.add_argument('-r', '--random_seeds', action='store_true')
    parser.add_argument('-a', '--all_logs', action='store_true')
    parser.add_argument('-w', '--weight_matrices', action='store_true')
    args = parser.parse_args()

    if args.max_and_jules:
        max_and_jules()
    if args.compute_similarity_dynamics:
        pass
    if args.random_seeds:
        run_for_random_seeds()
    if args.all_logs:
        run_for_all_log_files()
    if args.weight_matrices:
        pass


# %%
