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

# diffusion policy import
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler
from diffusers.training_utils import EMAModel
from diffusers.optimization import get_scheduler

# Painting imports
from style.diffusion_policy_gml.dataset import GmlDatasetNoSliding
from style.diffusion_policy_gml.network_transformer import Transformer1d, Transpose
import style.diffusion_policy_gml.network as network
import style.diffusion_policy_gml.utils as utils


class Editor:

    def __init__(
        self,
        RUN_FOLDER=Path('runs/Jul08_01-18-24_eagle'),
        # dataset_path="data/gml_by_stroke_PRESERVE_ASPECT_CENTERED_003000.zarr"
        dataset_path="data/gml_by_stroke_PRESERVE_ASPECT_CENTERED_000000.zarr"
    ):
        self.RUN_FOLDER = RUN_FOLDER
        self.dataset_path = dataset_path
        self.setup()
        self.load_dataset()
        self.load_model_and_noise_scheduler()

    def setup(self):
        ## Setup
        with open(self.RUN_FOLDER / 'network_kwargs.json', 'r') as f:
            network_kwargs_prelim = json.load(f)
        self.PRED_HORIZON = network_kwargs_prelim['horizon']
        self.PREDICTION_DIM = network_kwargs_prelim['output_dim']
        self.NUM_DIFFUSION_ITERS = 100
        self.device = torch.device('cuda')
        print(
            f'{self.PRED_HORIZON=}, {self.PREDICTION_DIM=}, {self.NUM_DIFFUSION_ITERS=}'
        )

    def load_dataset(self):
        with gerry.Stopwatch("Loading dataset"):
            self.dataset = GmlDatasetNoSliding(
                dataset_path=self.dataset_path,
                sequence_length=self.PRED_HORIZON,
                action_delta=True,
                action_penlift=True,
                ignore_jump_actions=True,
                normalize=dict(obs=False, action=True),
                prescale_obs=(-3, 3),
                prescale_penlift=(0, 3),
                max_drawings=1,
                min_traj_length=15)

    def load_model_and_noise_scheduler(self):
        ## Diffusion Setup

        # Noise scheduler
        noise_scheduler = DDPMScheduler(
            num_train_timesteps=self.NUM_DIFFUSION_ITERS,
            beta_schedule='squaredcos_cap_v2',
            clip_sample=True,
            clip_sample_range=3,
            prediction_type='epsilon')
        # Load the network_kwargs
        n_emb = 768
        InputEmbedding = nn.Sequential(Transpose(1, 2),
                                       nn.Conv1d(3, 15, 5, padding=2),
                                       nn.ReLU(),
                                       nn.Conv1d(15, n_emb, 5, padding=2),
                                       nn.ReLU(), Transpose(1, 2))

        _, network_kwargs = Transformer1d.FromJson(
            self.RUN_FOLDER / 'network_kwargs.json', InputEmbedding)
        for k, v in network_kwargs.items():
            print(f'{k:<20}', v)

        def load_model(checkpoint=None):
            fname = f'ema_noise_pred_net_{checkpoint}.pth' if checkpoint is not None else 'ema_noise_pred_net.pth'
            ema_noise_pred_net = Transformer1d(**network_kwargs)
            ema_noise_pred_net.to(self.device)
            ema_noise_pred_net.load_state_dict(
                torch.load(self.RUN_FOLDER / fname))
            return ema_noise_pred_net

        ema_noise_pred_net = load_model()

        self.ema_noise_pred_net = ema_noise_pred_net
        self.noise_scheduler = noise_scheduler

    def edit(self,
             strokes,
             t_start=10,
             repeat=1,
             seed=8675309,
             guidance_weight=4e4):
        ## Inference/Editing
        B = len(strokes)
        x = torch.zeros(B, self.PRED_HORIZON, 3)
        for i, stroke in enumerate(strokes):
            n = stroke.shape[0]
            while n > 128:
                stroke = stroke[::2]
                n = stroke.shape[0]
            xy = (stroke[:, 1:3] - 0.5) * 6
            x[i, :n, :2] = torch.tensor(self.dataset.normalize_obs(xy))
            x[i, n - 1, 2] = 1

        x = x.to(self.device)
        x_new = x * 1

        def loss(y):
            l1 = torch.nn.MSELoss()(y, x)
            l2 = torch.nn.MSELoss()(torch.diff(y, axis=1), torch.diff(x,
                                                                      axis=1))
            # return l1 + 3 * l2
            return l1

        guidance = network.guidance_fn(loss, weight=guidance_weight)

        torch.manual_seed(seed)
        history = [x_new * 1]
        for _ in tqdm.trange(repeat):
            x_noisy = network.add_noise(x_new, t_start, self.noise_scheduler)
            x_noisy[:, :, 2] = x[:, :, 2]
            if len(history) == 1:
                x_noisy_init = x_noisy * 1
            x_new = network.eval_partial(self.ema_noise_pred_net,
                                         self.noise_scheduler,
                                         x_noisy,
                                         t_start,
                                         guidance=guidance)
            history.append(x_new * 1)
        history = torch.stack(history, dim=0)

        return Editor.batched_to_strokes(x_new)

    @staticmethod
    def batched_to_strokes(x):
        x = utils.kill_after_penlift(x.detach().cpu().numpy(), -1)
        strokes = []
        for i in range(x.shape[0]):
            xyp = x[i]
            xyp[:, :2] = xyp[:, :2] / 6 + 0.5
            strokes.append(xyp[~np.isnan(xyp[:, 0]), :])
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

    with gerry.Stopwatch("Editing"):
        print(drawing.strokes)
        print('*' * 80)
        print(editor.edit(drawing.strokes))
