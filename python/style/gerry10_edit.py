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
from gerry11_all_networks import Network, NOISE_SCHEDULER, ClassifierFree1, DEVICE


class DefaultGuidanceLoss(nn.Module):

    def __init__(self, x, eta=100, eta_delta=0.25, eta_penup=1, W=1):
        super().__init__()

        assert x.shape[-1] == 5, "x should have shape (B, T, 5)"
        self.x = x
        self.T = x.shape[1]
        self.mse = torch.nn.MSELoss(reduction='none')
        self.eta = eta
        self.eta_delta = eta_delta
        self.eta_penup = eta_penup
        self.channel_weights = torch.Tensor([
            1, 1, eta_delta, eta_delta, eta_penup
        ]).to(x).reshape(1, 1, 5) * eta
        if isinstance(W, (int, float)):
            W = torch.ones((1, self.T, 1), device=x.device) * W
        elif len(W.shape) == 1:
            W = W[None, :, None]
        assert W.shape == (
            1, self.T,
            1), f"W should have shape (1, T={self.T}, 1), " + str(W.shape)
        self.W = W

    def forward(self, y):
        y = y[:, :self.T, :]
        l = torch.sum(self.mse(y, self.x) * self.W * self.channel_weights,
                      dim=(1, 2))
        return l

    def guidance_fn(self):

        def guidance(x):
            # Returns grad of loss function w.r.t. x
            # copy x and require grad
            x = x.clone().detach().to(x.device)
            x.requires_grad = True
            # compute loss
            with torch.enable_grad():
                loss = self(x)
            # compute grad
            grad = torch.autograd.grad(loss, x)[0]
            return grad

        return guidance


class EditorCnn:

    def __init__(self, model: Network):
        self.model = model
        self.device = torch.device('cuda')

    def edit(self,
             strokes,
             t_start=3,
             repeat=20,
             seed=8675309,
             guidance=None,
             guidance_loss_kwargs=dict(),
             loss=None,
             history=None,
             global_cond=None,
             global_cond_weight=1.3,
             print_progress=True):
        if global_cond is None and isinstance(self.model, ClassifierFree1):
            B = 1
            global_cond = torch.ones(
                (B, 1), device=DEVICE) * global_cond_weight

        # Create x
        x = self.model.network_input_from_strokes(strokes)
        T = x.shape[1]
        x_orig_pad = Editor.pad_to_8(x)
        obs_orig, act_orig = self.model.obs_act_from_network_output(x)
        if guidance is None:
            guidance = DefaultGuidanceLoss(
                x, **guidance_loss_kwargs).guidance_fn()

        x_new = x * 1
        for _ in tqdm.trange(repeat) if print_progress else range(repeat):
            x_new = Editor.pad_to_8(x_new)
            x_noisy = network.add_noise(x_new, t_start, NOISE_SCHEDULER)
            x_noisy[:, :, -1] = x_orig_pad[:, :, -1]
            x_new = network.eval_partial(self.model.ema_noise_pred_net,
                                         NOISE_SCHEDULER,
                                         x_noisy,
                                         t_start,
                                         guidance=guidance,
                                         global_cond=global_cond)
            x_new = x_new[:, :T, :]
            if history is not None:
                history.append(x_new * 1)
            x_new[:, :, -1] = x[:, :, -1]

        obs, act = self.model.obs_act_from_network_output(x_new)
        return obs, act

    @staticmethod
    def obs_act_to_strokes(obs, act, obs_orig):
        pen_up = act[:, 2] > 0.5

        x0 = obs[0]
        obs = np.concatenate(([[0, 0]], np.cumsum(action[:, :2], axis=0))) + x0

        strokes = []
        s = -1
        for i in np.argwhere(pen_up).flatten().tolist() + [len(pen_up)]:
            stroke = obs[s + 1:i + 1]
            if True:  # re-center stroke to original stroke's position
                stroke -= np.mean(stroke, axis=0) - np.mean(
                    obs_orig[s + 1:i + 1], axis=0)
            # print(np.mean(stroke, axis=0),
            #       np.mean(obs_orig[s + 1:i + 1], axis=0))
            strokes.append(stroke)
            s = i

        strokes = [
            np.concatenate([stroke, np.zeros((stroke.shape[0], 1))], axis=1)
            for stroke in strokes
        ]

        return strokes


class Editor:
    """OLD! do not use!"""

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
            if True:  # re-center stroke to original stroke's position
                stroke -= np.mean(stroke, axis=0) - np.mean(
                    obs_orig[s + 1:i + 1], axis=0)
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
