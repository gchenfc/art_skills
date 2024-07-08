# %% [markdown]
# First pass at using a transformer for diffusion model
# This script is for hyperparameter experiments

import matplotlib as mpl

mpl.use('Agg')

# %%
# General
from typing import Tuple, Sequence, Dict, Union, Optional
import numpy as np
import json
import torch
import torch.nn as nn
from tqdm.auto import tqdm
import matplotlib.pyplot as plt
from pathlib import Path
from torch.utils.tensorboard import SummaryWriter
import gerry

# diffusion policy import
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler
from diffusers.training_utils import EMAModel
from diffusers.optimization import get_scheduler

# Painting imports
import cv2
from style.diffusion_policy_gml.dataset import GmlDatasetNoSliding
from style.diffusion_policy_gml.network import compute_noise, compute_orig
import style.diffusion_policy_gml.network as network
from style.diffusion_policy_gml.network_transformer import Transformer1d
import style.diffusion_policy_gml.utils as utils

# %%
PRED_HORIZON = 512
PREDICTION_DIM = 5  # [x, y, dx, dy, penup]
PREDICTION_DIM = 3  # [x, y, penup]
NUM_DIFFUSION_ITERS = 100

# %% [markdown]
# ## Load Dataset

# %%
dataset_path = "data/gml_by_drawing_PRESERVE_ASPECT_CENTERED_003000.zarr"

with gerry.Stopwatch("Loading dataset"):
    dataset = GmlDatasetNoSliding(dataset_path=dataset_path,
                                  sequence_length=PRED_HORIZON,
                                  action_delta=True,
                                  action_penlift=True,
                                  normalize=dict(obs=False, action=True),
                                  max_drawings=1)
print(f'The number of drawings is {len(dataset.episode_ends)}')
print(f'The number of training samples is like', dataset.indices.shape)

# create dataloader
with gerry.Stopwatch("Creating dataloader"):
    dataloader = torch.utils.data.DataLoader(dataset,
                                             batch_size=128,
                                             num_workers=1,
                                             shuffle=True,
                                             pin_memory=True,
                                             persistent_workers=True)

# visualize data in batch
print("Num batches:          ", len(dataloader))
batch = next(iter(dataloader))
print("batch['obs'].shape:   ", batch['obs'].shape)
print("batch['action'].shape:", batch['action'].shape)

# %% [markdown]
# ## Diffusion Setup

# %%
network_kwargs = dict(
    input_dim=PREDICTION_DIM,
    output_dim=PREDICTION_DIM,
    horizon=PRED_HORIZON,
    n_layer=8,
    #    n_head=12,
    #    n_emb=768,
    n_head=8,
    n_emb=128,
    time_as_cond=False,
    use_sinusoidal_pos_embedding=True)

# %%
# Noise scheduler
noise_scheduler = DDPMScheduler(
    num_train_timesteps=NUM_DIFFUSION_ITERS,
    # the choise of beta schedule has big impact on performance
    # we found squared cosine works the best
    beta_schedule='squaredcos_cap_v2',
    # clip output to [-1,1] to improve stability
    clip_sample=True,
    clip_sample_range=3,
    # our network predicts noise (instead of denoised action)
    prediction_type='epsilon')

# %%
# Network!
noise_pred_net = Transformer1d(**network_kwargs)
# (Initialization handled in constructor)

# %%
# device transfer
device = torch.device('cuda')
_ = noise_pred_net.to(device)

# %% [markdown]
# ## Training

# %%
# num_epochs = 500 // len(dataloader) + 1
# num_epochs = 1500 // len(dataloader) + 1
num_epochs = 625 // len(dataloader) + 1

if True:
    noise_pred_net.apply(noise_pred_net._init_weights)
pos_emb_bak = noise_pred_net.pos_emb.cpu().detach().numpy()

ema = EMAModel(parameters=noise_pred_net.parameters(),
               model=noise_pred_net,
               power=0.75)

optimizer = noise_pred_net.configure_optimizers(learning_rate=1e-3,
                                                weight_decay=1e-3)

lr_scheduler = get_scheduler(name='cosine',
                             optimizer=optimizer,
                             num_warmup_steps=500,
                             num_training_steps=len(dataloader) * num_epochs)

all_losses = list()

writer = SummaryWriter()  # Tensorboard

print(writer.log_dir)

# %%
first_iter = True
try:
    starting_step = len(all_losses)
    with tqdm(range(num_epochs), desc='Epoch') as tglobal:
        # epoch loop
        for epoch_idx in tglobal:
            epoch_loss = list()
            # with tqdm(dataloader, desc='Batch') as tdataloader:
            if True:
                tdataloader = dataloader
                # batch loop
                for batch_n in tdataloader:
                    # Extract data
                    obs_n = (batch_n['obs'].to(device) -
                             0.5) * 2  # scale [-1, 1]
                    action_n = batch_n['action'].to(
                        device)  # - 0.1 # scale [-0.1, 0.9]
                    # obs_n = (dummy_obs.to(device) - 0.5) * 1
                    # action_n = dummy_action.to(device) - 0.5
                    # x = torch.cat([obs_n, action_n[..., -1:]], dim=-1)# * 50 / 3
                    x = torch.cat([obs_n, action_n[..., -1:]], dim=-1) * 3
                    x = x.repeat(32, 1, 1)
                    B = x.shape[0]
                    global_cond = None

                    # sample noise to add to actions
                    noise = torch.randn(x.shape, device=device)
                    timesteps = torch.randint(
                        0,
                        noise_scheduler.config.num_train_timesteps, (B, ),
                        device=device).long()
                    noisy_x = noise_scheduler.add_noise(x, noise, timesteps)
                    # noisy_x = network.binarize_last(noisy_x)

                    # predict the noise residual
                    noise_pred = noise_pred_net(noisy_x,
                                                timesteps,
                                                global_cond=global_cond)

                    # L2 loss
                    # noise[..., -1] = noisy_x[..., -1] - x[..., -1]
                    loss = nn.functional.mse_loss(noise_pred, noise)
                    # loss += nn.functional.mse_loss(torch.diff(noise_pred, axis=1), torch.diff(noise, axis=1))

                    # optimize
                    loss.backward()
                    optimizer.step()
                    optimizer.zero_grad()
                    lr_scheduler.step()

                    ema.step(noise_pred_net)

                    # logging
                    loss_cpu = loss.item()
                    # tdataloader.set_postfix(loss=loss_cpu)
                    epoch_loss.append(loss_cpu)
                    writer.add_scalar('Loss',
                                      loss_cpu,
                                      global_step=starting_step +
                                      len(all_losses) + len(epoch_loss))

                    if first_iter:
                        gerry.GpuStats.print_model_memory(noise_pred_net,
                                                          verbosity=0,
                                                          print_unknown=True,
                                                          prefix='| ')
                        first_iter = False

            tglobal.set_postfix(loss=np.mean(epoch_loss))
            all_losses.extend(epoch_loss)
            writer.add_scalar('Loss/Epoch',
                              np.mean(epoch_loss),
                              global_step=epoch_idx)
except KeyboardInterrupt:
    all_losses.extend(epoch_loss)
    pass
writer.add_text('network_kwargs', json.dumps(network_kwargs))
# writer.close()
json.dump(network_kwargs, open(f'{writer.log_dir}/network_kwargs.json', 'w'))

print(writer.log_dir)

# %%
# Weights of the EMA model is used for inference
ema_noise_pred_net = Transformer1d(**network_kwargs)
ema_noise_pred_net.to(device)
ema.copy_to(ema_noise_pred_net.parameters())
torch.save(ema_noise_pred_net.state_dict(),
           f'{writer.log_dir}/ema_noise_pred_net.pth')
# torch.jit.save(torch.jit.script(ema_noise_pred_net), f'{writer.log_dir}/ema_noise_pred_net.pt')

# Print the log directory where the weights are saved
print(writer.log_dir)

# Plot the loss
plt.figure(figsize=(5, 1.5))
plt.semilogy(all_losses)
plt.title('Loss')
plt.savefig(f'{writer.log_dir}/loss.svg')

# %% [markdown]
# ## Inference

# %%
# Full generation

# B = 15  # num samples
# B = 6*6  # num samples
B = 1  # num samples
all_obs = {}
all_actions = {}
all_histories = {}

horizon = PRED_HORIZON

x_init = torch.randn((B, horizon, PREDICTION_DIM), device=device)
history = []

x_out = network.eval(
    ema_noise_pred_net,
    noise_scheduler,
    x_init,
    # x_out = network.eval(noise_pred_net, noise_scheduler, x_init,
    global_cond=global_cond[[0]] if global_cond is not None else None,
    log_history=history)

x_out = x_out.detach().cpu().numpy()
action = dataset.unnormalize_action(x_out[..., -3:])

all_actions[horizon] = action
all_histories[horizon] = history
all_obs[horizon] = dataset.unnormalize_obs(x_out[..., :-3])

# %%
x_ = x.detach().cpu().numpy()

fig, axes = utils.plot_result(dataset,
                              x_out[0, :, 2:],
                              x_out[0, :, :2],
                              clean=False)
fig.suptitle(
    f'{network_kwargs["n_layer"]} layers, {network_kwargs["n_head"]} heads, {network_kwargs["n_emb"]} emb',
    fontsize=24)
plt.savefig(f'{writer.log_dir}/output_example.svg')
np.savez(f'{writer.log_dir}/output_example.npz', x_out=x_out, x_train=x_)

fig, axes = utils.plot_result(dataset,
                              x_[0, :, 2:],
                              x_[0, :, :2],
                              clean=False,
                              axes=axes,
                              traj_line_kwargs=dict(line_ls='k--'))
plt.savefig(f'{writer.log_dir}/output_and_input_example.svg')
# fig, axes = utils.plot_result(dataset, x_[0, :, 2:] / 3 + 0.5, x_[0, :, :2], clean=False, traj_line_kwargs=dict(line_ls='k--'));
# utils.plot_result(dataset, x_out[0, :, 2:] / 3 + 0.5, x_out[0, :, :2], clean=False, axes=axes);
writer.add_figure('output_example', fig)

writer.close()
