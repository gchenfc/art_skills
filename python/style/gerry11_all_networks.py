#%% Imports

import dataclasses
from typing import Optional, Callable
from pathlib import Path
from functools import lru_cache

import torch
import numpy as np
import cv2
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
from gerry import Stopwatch

import style.diffusion_policy_gml.network as network
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler
from style.diffusion_policy_gml.network import ConditionalUnet1D
from style.diffusion_policy_gml.dataset import GmlDataset
import style.diffusion_policy_gml.utils as utils

DT = 0.02
NUM_DIFFUSION_ITERS = 100
PRED_HORIZON = 512  # should only be used for dataset
DEVICE = torch.device('cuda') if torch.cuda.is_available() else torch.device(
    'cpu')


#%% Load Dataset
class HDict(dict):

    def __hash__(self):
        return hash(frozenset(self.items()))


@lru_cache
def load_dataset(kwargs):
    dataset = GmlDataset(**kwargs)
    dataset.stats = {
        k: {
            k_: torch.from_numpy(v_).to(DEVICE).detach().requires_grad_(False)
            for k_, v_ in v.items()
        }
        for k, v in dataset.stats.items()
    }
    return dataset


#%% Configurations


@dataclasses.dataclass
class Args:
    run_folder: str
    action_dim: int
    cond_dim: int
    global_cond: Optional[torch.Tensor] = None
    guidance: Optional[Callable] = None
    dataset_path: Path = Path(
        "data/toppra_by_drawing_PRESERVE_ASPECT_CENTERED_003000.zarr")


NET_ARGS = dict(
    decoupled=Args(run_folder='runs/Apr04_21-01-28_eagle',
                   action_dim=5,
                   cond_dim=0),
    guidance=Args(run_folder='runs/Apr04_21-01-28_eagle',
                  action_dim=5,
                  cond_dim=0,
                  guidance='please fill this in'),
    finetuned=Args(run_folder='runs/May28_20-06-01_eagle',
                   action_dim=5,
                   cond_dim=0),
    control=Args(run_folder='runs/May29_05-00-04_eagle',
                 action_dim=3,
                 cond_dim=0),
    classifier_free=Args(
        run_folder='runs/May28_20-34-26_eagle',
        action_dim=5,
        cond_dim=1,
        global_cond=1.3,
        dataset_path=Path(
            "data/toppra_by_drawing_PRESERVE_ASPECT_CENTERED_003000.zarr")),
)

NOISE_SCHEDULER = DDPMScheduler(
    num_train_timesteps=NUM_DIFFUSION_ITERS,
    # the choise of beta schedule has big impact on performance
    # we found squared cosine works the best
    beta_schedule='squaredcos_cap_v2',
    # clip output to [-1,1] to improve stability
    clip_sample=True,
    clip_sample_range=5,
    # our network predicts noise (instead of denoised action)
    prediction_type='epsilon')

#%% Networks


class Network:

    def __init__(self, args: Args):
        self.args = args
        if True:
            self.dataset = load_dataset(
                HDict(
                    dataset_path=args.dataset_path,
                    sequence_length=PRED_HORIZON,
                    pad_before=0,
                    pad_after=0,
                    # stride=10,
                    action_delta=True,
                    action_penlift=True,
                    normalize=HDict(obs=False, action=True),
                    # max_drawings=100
                ))
        self.load_net()
        torch.manual_seed(8675309)

    def load_net(self):
        self.ema_noise_pred_net = ConditionalUnet1D(
            input_dim=self.args.action_dim,
            global_cond_dim=self.args.cond_dim,
        )
        self.ema_noise_pred_net.to(DEVICE)
        self.ema_noise_pred_net.load_state_dict(
            torch.load(f'{self.args.run_folder}/ema_noise_pred_net.pth'))

        return self.ema_noise_pred_net

    def eval_network(self,
                     B: int,
                     horizon: int,
                     history=None,
                     global_cond=None):
        action_n_init = torch.randn((B, horizon, self.args.action_dim),
                                    device=DEVICE)

        action_n = network.eval(self.ema_noise_pred_net,
                                NOISE_SCHEDULER,
                                action_n_init,
                                global_cond=global_cond if global_cond
                                is not None else self.args.global_cond,
                                guidance=self.args.guidance,
                                log_history=history)

        return action_n

    def obs_act_from_network_output(self, action_n):
        if self.args.action_dim != 5:
            raise NotImplementedError(
                'Only implemented for action_dim=5.  Please implement ')
        obs_n, act_n = torch.split(action_n, [2, 3], dim=-1)
        return (self.dataset.unnormalize_obs(obs_n),
                self.dataset.unnormalize_action(act_n))

    def generate(self, B: int, horizon: int, global_cond=None):
        action_n = self.eval_network(B, horizon, global_cond=global_cond)
        obs, act = self.obs_act_from_network_output(action_n)
        return obs, act

    def generate_img(self, B: int, horizon: int, global_cond=None):
        obs, act = self.generate(B, horizon, global_cond=global_cond)
        return self.render_img(obs)

    @staticmethod
    def render_img(obs, size=256, center=True):
        if isinstance(obs, torch.Tensor):
            obs = obs.detach().cpu().numpy()
        if isinstance(obs, list) or len(obs.shape) == 3:
            return np.stack([Network.render_img(obs_) for obs_ in obs])

        if center:
            obs = obs - np.mean(obs, axis=0, keepdims=True) + 0.5

        img = np.zeros((size, size, 3), dtype=np.uint8)
        return cv2.polylines(img,
                             [(np.clip(obs, 0, 1) * size).astype(np.int32)],
                             isClosed=False,
                             color=(255, 255, 255),
                             thickness=5,
                             lineType=cv2.LINE_AA)


class Base(Network):

    def __init__(self):
        super().__init__(NET_ARGS['decoupled'])


class Decoupled(Network):

    def __init__(self):
        super().__init__(NET_ARGS['decoupled'])

    def obs_act_from_network_output(self, action_n):
        obss, acts = super().obs_act_from_network_output(action_n)
        all_obs, all_act = [], []
        for i, (obs, act) in enumerate(zip(obss, acts)):
            obs, pen_up = Retime.fix_obs(act, obs)
            t, x, v, a, pen_up = Retime.retime(pen_up, obs)
            all_obs.append(x)
            all_act.append(torch.concatenate((v * DT, pen_up[:, None]),
                                             axis=1))
        return all_obs, all_act


class Finetuned(Network):

    def __init__(self):
        super().__init__(NET_ARGS['finetuned'])


class ClassifierFree1(Network):

    def __init__(self):
        super().__init__(NET_ARGS['classifier_free'])

    def eval_network(self, B: int, horizon: int):
        return super().eval_network(B,
                                    horizon,
                                    global_cond=torch.ones(
                                        (B, 1), device=DEVICE) * 1.3)


class Guidance(Network):
    """Done as long as guidance function is added to ARGS"""

    def __init__(self):
        super().__init__(NET_ARGS['guidance'])
        self.args.guidance = GuidanceFunction(self.dataset).loss_fn


class Control(Network):

    def __init__(self):
        super().__init__(NET_ARGS['control'])

        act_scale = self.dataset.unnormalize_action(
            torch.tensor([1., 1., 0.], device=DEVICE, dtype=torch.float32))
        self.parameterize = ControlParameterization(act_scale)

    def eval_network(self, B: int, horizon: int):
        action_n_init = torch.randn((B, horizon, self.args.action_dim),
                                    device=DEVICE)

        activations_init = self.parameterize.action_n2acc_activations(
            action_n_init)
        activations = network.eval(self.ema_noise_pred_net,
                                   NOISE_SCHEDULER,
                                   action_n_init,
                                   global_cond=None)
        action_n = self.parameterize.acc_activations2action_n(activations)

        return action_n

    def obs_act_from_network_output(self, action_n):
        act = self.dataset.unnormalize_action(action_n)
        act = utils.batchify(ControlParameterization.correct_actions)(act)

        obs = torch.cumsum(torch.concatenate((torch.zeros(
            (act.shape[0], 1, 2), device=DEVICE), act[:, :-1, :2]),
                                             dim=1),
                           dim=1)
        obs -= torch.mean(obs, axis=1, keepdim=True)
        obs += 0.5

        return obs, act


#%% Util classes/functions
class GuidanceFunction:

    def __init__(self, dataset):
        self.dataset = dataset

    # def guidance_loss():
    #     action_stats = {
    #         k: torch.from_numpy(v).to(DEVICE)
    #         for k, v in dataset.stats['action'].items()
    #     }
    #     for v in action_stats.values():
    #         v.requires_grad = False

    # def unnormalize_action(data):
    #     return normalize_data(
    #         data, action_stats, center=not dataset.action_delta
    #     ) if dataset.normalize['action'] else data

    def vel_and_acc(action_unnorm):
        vel = action_unnorm[:, :, :2] / DT
        acc = torch.diff(vel, axis=1) / DT
        return vel, acc

    def loss_fn(self, traj):
        VEL_MAX = 0.2
        ACC_MAX = 0.4
        # traj has shape [batch, pred_horizon, action_dim]
        act = self.dataset.unnormalize_action(traj[:, :, 2:])
        vel, acc = GuidanceFunction.vel_and_acc(act)
        # ignore travel strokes
        penup = act[:, :, 2] > 0.5
        vel[penup] = 0
        acc[penup[:, :-1] | penup[:, 1:]] = 0
        loss = (torch.nn.functional.relu(torch.abs(acc) - ACC_MAX).mean() +
                torch.nn.functional.relu(torch.abs(vel) - VEL_MAX).mean())
        return loss


class Retime:

    def fix_obs(act, obs):
        pen_up = act[:, 2] > 0.5
        obs = torch.concatenate((torch.zeros(
            (1, 2), device=DEVICE), torch.cumsum(act[:-1, :2],
                                                 axis=0))) + obs[0]
        return obs, pen_up

    def break_up(pen_up, obs):
        breaks = np.where(pen_up)[0]
        ret = []
        start = 0
        for b in breaks + 1:
            ret.append(obs[start:b])
            start = b
        ret.append(obs[start:])
        return ret

    def retime(pen_up, obs):

        def retime_one(obs):
            t = np.arange(0, obs.shape[0]) * DT
            path = ta.SplineInterpolator(t, obs)
            t_alt = np.arange(0, t[-1], DT / 100)
            traj_alt = path.eval(t_alt)
            path = ta.SplineInterpolator(t_alt, traj_alt)
            pc_vel = constraint.JointVelocityConstraint(
                np.array([[-0.2, 0.2], [-0.2, 0.2]]))
            pc_acc = constraint.JointAccelerationConstraint(
                np.array([[-0.4, 0.4], [-0.4, 0.4]]),
                discretization_scheme=constraint.DiscretizationType.
                Interpolation)
            instance = algo.TOPPRA([pc_vel, pc_acc],
                                   path,
                                   solver_wrapper='seidel')
            jnt_traj = instance.compute_trajectory(0, 0)

            tnew = np.arange(0, jnt_traj.duration, DT)
            return tnew, jnt_traj.eval(tnew), jnt_traj.evald(
                tnew), jnt_traj.evaldd(tnew)

        retime_one = utils.wrap_numpy_fn(retime_one)

        t, x, v, a = [], [], [], []
        pen_up = []

        t0 = 0
        for stroke in Retime.break_up(pen_up, obs):
            t_, x_, v_, a_ = retime_one(stroke)
            t.append(t_ + t0)
            x.append(x_)
            v.append(v_)
            a.append(a_)
            pen_up.append(t_ == t_[-1])
            t0 += t_[-1] + DT

        return (torch.concatenate(t, axis=0), torch.concatenate(x, axis=0),
                torch.concatenate(v, axis=0), torch.concatenate(a, axis=0),
                torch.concatenate(pen_up, axis=0))


class ControlParameterization:

    def __init__(self, act_scale):
        self.act_scale = act_scale

    # Correct the velocities
    def correct_actions(action):
        dx = action[:, :2]
        for i in range(2):
            for t in range(dx.shape[0]):
                if dx[t, i] > (0.2 * DT):
                    dx[t:, i] -= dx[t, i] - 0.2 * DT
                elif dx[t, i] < (-0.2 * DT):
                    dx[t:, i] += -0.2 * DT - dx[t, i]
        action[:, :2] = dx
        return action

    def activate(output):
        return torch.tanh(output) * 0.4

    def deactivate(acc):
        return torch.atanh(torch.clip(acc / 0.4, -1, 1))

    def action_n2acc_activations(self, action):
        # action is [B, T, 3]
        # unnormalize
        v = action[:, :, :2] * self.act_scale[:2] / DT
        a = torch.diff(v, axis=1, append=v[:, -1:, :]) / DT
        return torch.concatenate(
            (ControlParameterization.deactivate(a), action[:, :, 2:]), dim=-1)

    def acc_activations2action_n(self, output):
        # output is [B, T, 3]
        # normalize
        a = ControlParameterization.activate(output[..., :2])
        # v = torch.cumsum([0] + a, axis=1) * DT
        # TypeError: can only concatenate list (not "Tensor") to list
        # v = torch.cumsum(torch.cat([torch.zeros((output.shape[0], 1, 2), device=output.device), a], dim=1), dim=1) * DT
        v = torch.cumsum(a, dim=1) * DT
        normed = v / self.act_scale[:2] * DT
        return torch.concatenate((normed, output[:, :, 2:]), dim=-1)


#%% main
def main():
    with Stopwatch('Testing Constructors', print_start_and_end=True):
        with Stopwatch('\tDecoupled'):
            decoupled = Decoupled()
        with Stopwatch('\tFinetuned'):
            finetuned = Finetuned()
        with Stopwatch('\tClassifierFree1'):
            classifier_free = ClassifierFree1()
        with Stopwatch('\tGuidance'):
            guidance = Guidance()
        with Stopwatch('\tControl'):
            control = Control()

    B = 10
    with Stopwatch("Testing evaluations", print_start_and_end=True):
        for model in [
                decoupled, finetuned, classifier_free, guidance, control
        ]:
            with Stopwatch(f'\t{model.__class__.__name__}'):
                import tqdm.auto as tqdm
                for iter in tqdm.trange(10):
                    action_n = model.eval_network(B, 128)
                    obs, act = model.obs_act_from_network_output(action_n)
                    imgs = model.render_img(obs)
                    for i, img in enumerate(imgs):
                        cv2.imwrite(
                            f'results/gerry11_all_networks/{model.__class__.__name__}/{model.__class__.__name__}_{i + iter * 10:02d}.png',
                            img)


if __name__ == '__main__':
    main()

# %%
