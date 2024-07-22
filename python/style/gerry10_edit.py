# General
from typing import Tuple, Sequence, Dict, Union, Optional
import numpy as np
import json
import torch
import torch.nn as nn
import tqdm.auto as tqdm
import matplotlib.pyplot as plt
from pathlib import Path
import gerry
import pickle
from collections import namedtuple

# diffusion policy import
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler
from diffusers.training_utils import EMAModel
from diffusers.optimization import get_scheduler

# Painting imports
from style.diffusion_policy_gml.dataset import GmlDataset
from style.diffusion_policy_gml.network import ConditionalUnet1D
import style.diffusion_policy_gml.network as network
import style.diffusion_policy_gml.utils as utils


class Editor:

    def __init__(
        self,
        RUN_FOLDER=Path('runs/Apr04_21-01-28_eagle'),
        dataset_path="data/gml_by_drawing_PRESERVE_ASPECT_CENTERED_003000.zarr"
        # dataset_path="data/gml_by_stroke_PRESERVE_ASPECT_CENTERED_000000.zarr"
    ):
        self.RUN_FOLDER = RUN_FOLDER
        self.NUM_DIFFUSION_ITERS = 100
        self.device = torch.device('cuda')

        self.dataset_path = dataset_path
        self.load_dataset()
        self.load_model_and_noise_scheduler()

        self.past_strokes = []

    def load_dataset(self):
        with gerry.Stopwatch("Loading dataset"):
            self.dataset = GmlDataset(
                dataset_path=self.dataset_path,
                sequence_length=512,
                pad_before=0,
                pad_after=0,
                # stride=10,
                action_delta=True,
                action_penlift=True,
                normalize=dict(obs=False, action=True),
                # max_drawings=100
            )

    def load_model_and_noise_scheduler(self):
        ## Diffusion Setup

        # Noise scheduler
        noise_scheduler = DDPMScheduler(
            num_train_timesteps=self.NUM_DIFFUSION_ITERS,
            beta_schedule='squaredcos_cap_v2',
            clip_sample=True,
            clip_sample_range=3,
            prediction_type='epsilon')

        # Load model
        def load_model(checkpoint=None):
            fname = f'ema_noise_pred_net_{checkpoint}.pth' if checkpoint is not None else 'ema_noise_pred_net.pth'
            ema_noise_pred_net = ConditionalUnet1D(input_dim=5,
                                                   global_cond_dim=0)
            ema_noise_pred_net.to(self.device)
            ema_noise_pred_net.load_state_dict(
                torch.load(self.RUN_FOLDER / fname))
            return ema_noise_pred_net

        ema_noise_pred_net = load_model()

        self.ema_noise_pred_net = ema_noise_pred_net
        self.noise_scheduler = noise_scheduler

    @staticmethod
    def pad_to_8(x):
        B, _, C = x.shape
        n = 8 - (x.shape[1] % 8)
        # print(n, x[0, -8:, :])
        tmp = torch.cat(
            [x, x[:, -1, :] + torch.zeros((B, n, C), device=x.device)], dim=1)
        tmp[:, -n:, 2:] = 0
        # print(n, tmp[0, -8:, :])
        return tmp

    def edit(self,
             strokes,
             t_start=10,
             repeat=1,
             seed=8675309,
             guidance_weight=4e4,
             loss=None,
             history=None,
             print_progress=True):
        self.past_strokes.append(strokes[0])

        # Create x
        B = 1
        assert len(strokes) == 1, "Only one stroke is supported at the moment"
        drawing = namedtuple('Drawing', ['strokes'])
        # drawing.strokes = self.past_strokes
        drawing.strokes = strokes
        obs_n, act_n = self.dataset.create_normalized_from_drawing(drawing)

        obs_n, act_n = torch.tensor(obs_n), torch.tensor(act_n)
        x = torch.concatenate([obs_n, act_n], dim=1).to(self.device)[None, ...]

        # # Pad with 0s at the end
        # n = 8 - (x.shape[1] % 8)
        # # x = torch.cat([x, torch.zeros((B, n, 5), device=self.device)], dim=1)
        # # x[:, -n:, :2] = x[:, -n - 1, :2]
        # x = torch.cat(
        #     [x, x[:, -1, :] + torch.zeros((B, n, 5), device=self.device)],
        #     dim=1)
        # # print(x)
        T = x.shape[1]

        x_new = x * 1

        def loss_(y):
            if loss is None:
                y = y[:, :T, :]
                l1 = torch.nn.MSELoss()(y, x)
                l2 = torch.nn.MSELoss()(y[:, :, :2], x[:, :, :2])
                # l2 = torch.nn.MSELoss()(torch.diff(y, axis=1), torch.diff(x,
                #                                                           axis=1))
                return l1 + 3 * l2
                return 1 * l1 + 10 * l2
                return 10 * l1 + 10 * l2
            else:
                return loss(x, y[:, :T, :])

        guidance = network.guidance_fn(loss_, weight=guidance_weight)

        torch.manual_seed(seed)
        if history is not None:
            history.append(x_new * 1)
        for _ in tqdm.trange(repeat) if print_progress else range(repeat):
            x_new = Editor.pad_to_8(x_new)
            x_noisy = network.add_noise(x_new, t_start, self.noise_scheduler)
            # x_noisy[:, :, 2] = x[:, :, 2]
            # if len(history) == 1:
            #     x_noisy_init = x_noisy * 1
            x_new = network.eval_partial(self.ema_noise_pred_net,
                                         self.noise_scheduler,
                                         x_noisy,
                                         t_start,
                                         guidance=guidance)
            x_new = x_new[:, :T, :]
            if history is not None:
                history.append(x_new * 1)
            x_new[:, :, 4] = x[:, :, 4]
            # x_new[:, :, 2] = x[:, :, 2]
        if history is not None:
            history = torch.stack(history, dim=0)

        return Editor.batched_to_strokes(x_new.detach().cpu().numpy()[0],
                                         self.dataset, drawing.strokes[0][0,
                                                                          1:3],
                                         x.detach().cpu().numpy()[0])

    @staticmethod
    def batched_to_strokes(x, dataset, x0, x_orig):
        action_n, obs_n = x[:, 2:], x[:, :2]

        pen_up = action_n[:, 2] > 0.5
        action = dataset.unnormalize_action(action_n)
        obs = dataset.unnormalize_obs(obs_n)
        obs_orig = dataset.unnormalize_obs(x_orig[:, :2])
        # print(x0, obs[0])
        # print(obs)
        # print(obs_n)
        x0 = obs[0]
        obs = np.concatenate(([[0, 0]], np.cumsum(action[:, :2], axis=0))) + x0

        strokes = []
        s = -1
        for i in np.argwhere(pen_up).flatten().tolist() + [len(pen_up)]:
            stroke = obs[s + 1:i + 1]
            # stroke -= np.mean(stroke, axis=0) - np.mean(obs_orig[s + 1:i + 1],
            #                                             axis=0)
            # print(np.mean(stroke, axis=0),
            #       np.mean(obs_orig[s + 1:i + 1], axis=0))
            strokes.append(stroke)
            s = i

        strokes = [
            np.concatenate([stroke, np.zeros((stroke.shape[0], 1))], axis=1)
            for stroke in strokes
        ]

        return strokes


if __name__ == '__main__':
    editor = Editor()

    # Read in a gml file
    import load_gml
    basename = 'shapes3'
    root = Path('data') / 'custom_gmls'
    infile = root / f'{basename}.json'
    drawing = load_gml.Drawing(infile,
                               scale_behavior='PRESERVE_ASPECT_CENTERED')

    drawing.strokes = drawing.strokes[:1]

    with gerry.Stopwatch("Editing"):
        print(drawing.strokes)
        print('*' * 80)
        print(editor.edit(drawing.strokes))
