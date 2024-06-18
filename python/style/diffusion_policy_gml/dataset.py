"""
### **Dataset**

Defines `PushTStateDataset` and helper functions

The dataset class
- Load data (obs, action) from a zarr storage
- Normalizes each dimension of obs and action to [-1,1]
- Returns
 - All possible segments with length `pred_horizon`
 - Pads the beginning and the end of each episode with repetition
 - key `obs`: shape (obs_horizon, obs_dim)
 - key `action`: shape (pred_horizon, action_dim)
"""

import torch
import numpy as np
import zarr
from collections import defaultdict
from typing import Optional
from load_gml import Drawing


def create_sample_indices(episode_ends: np.ndarray,
                          sequence_length: int,
                          pad_before: int = 0,
                          pad_after: int = 0):
    indices = list()
    for i in range(len(episode_ends)):
        start_idx = 0
        if i > 0:
            start_idx = episode_ends[i - 1]
        end_idx = episode_ends[i]
        episode_length = end_idx - start_idx

        min_start = -pad_before
        max_start = episode_length - sequence_length + pad_after

        # range stops one idx before end
        for idx in range(min_start, max_start + 1):
            buffer_start_idx = max(idx, 0) + start_idx
            buffer_end_idx = min(idx + sequence_length,
                                 episode_length) + start_idx
            start_offset = buffer_start_idx - (idx + start_idx)
            end_offset = (idx + sequence_length + start_idx) - buffer_end_idx
            sample_start_idx = 0 + start_offset
            sample_end_idx = sequence_length - end_offset
            indices.append([
                buffer_start_idx, buffer_end_idx, sample_start_idx,
                sample_end_idx
            ])
    indices = np.array(indices)
    return indices


def sample_sequence(train_data,
                    sequence_length,
                    buffer_start_idx,
                    buffer_end_idx,
                    sample_start_idx,
                    sample_end_idx,
                    action_delta=False):
    result = dict()
    for key, input_arr in train_data.items():
        sample = input_arr[buffer_start_idx:buffer_end_idx]
        data = sample
        if (sample_start_idx > 0) or (sample_end_idx < sequence_length):
            data = np.zeros(shape=(sequence_length, ) + input_arr.shape[1:],
                            dtype=input_arr.dtype)
            if sample_start_idx > 0:
                data[:sample_start_idx] = 0 if (
                    action_delta and key == 'action') else sample[0]
            if sample_end_idx < sequence_length:
                data[sample_end_idx:] = sample[-1]
            data[sample_start_idx:sample_end_idx] = sample
        result[key] = data
    return result


# normalize data
def get_data_stats(data):
    data = data.reshape(-1, data.shape[-1])
    stats = {'min': np.min(data, axis=0), 'max': np.max(data, axis=0)}
    return stats


def normalize_data(data, stats, center=False):
    if center:
        # nomalize to [0,1]
        ndata = (data - stats['min']) / (stats['max'] - stats['min'])
        # normalize to [-1, 1]
        ndata = ndata * 2 - 1
    else:
        ndata = data / (stats['max'] - stats['min']) * 2
    return ndata * 5


def unnormalize_data(ndata, stats, center=False):
    if center:
        ndata = (ndata / 5 + 1) / 2
        data = ndata * (stats['max'] - stats['min']) + stats['min']
    else:
        data = ndata / 10 * (stats['max'] - stats['min'])
    return data


# dataset
class PushTStateDataset(torch.utils.data.Dataset):
    def __init__(self,
                 dataset_path,
                 pred_horizon,
                 obs_horizon,
                 action_horizon,
                 action_delta=False):

        # read from zarr dataset
        dataset_root = zarr.open(dataset_path, 'r')
        # All demonstration episodes are concatinated in the first dimension N
        train_data = {
            # (N, action_dim)
            'action':
            dataset_root['data']['action' if action_delta else 'state']
            [:].astype(np.float32),
            # (N, obs_dim)
            'obs':
            dataset_root['data']['state'][:].astype(np.float32)
        }
        # Marks one-past the last index for each episode
        episode_ends = dataset_root['meta']['episode_ends'][:]

        # compute start and end of each state-action sequence
        # also handles padding
        indices = create_sample_indices(
            episode_ends=episode_ends,
            sequence_length=pred_horizon,
            # add padding such that each timestep in the dataset are seen
            pad_before=obs_horizon - 1,
            pad_after=action_horizon - 1)

        # compute statistics and normalized data to [-1,1]
        stats = dict()
        normalized_train_data = dict()
        for key, data in train_data.items():
            stats[key] = get_data_stats(data)
            normalized_train_data[key] = normalize_data(data, stats[key])

        self.indices = indices
        self.stats = stats
        self.normalized_train_data = normalized_train_data
        self.episode_ends = episode_ends
        self.pred_horizon = pred_horizon
        self.action_horizon = action_horizon
        self.obs_horizon = obs_horizon
        self.action_delta = action_delta

    def __len__(self):
        # all possible segments of the dataset
        return len(self.indices)

    def __getitem__(self, idx):
        # get the start/end indices for this datapoint
        buffer_start_idx, buffer_end_idx, \
            sample_start_idx, sample_end_idx = self.indices[idx]

        # get nomralized data using these indices
        nsample = sample_sequence(train_data=self.normalized_train_data,
                                  sequence_length=self.pred_horizon,
                                  buffer_start_idx=buffer_start_idx,
                                  buffer_end_idx=buffer_end_idx,
                                  sample_start_idx=sample_start_idx,
                                  sample_end_idx=sample_end_idx,
                                  action_delta=self.action_delta)

        # discard unused observations
        nsample['obs'] = nsample['obs'][:self.obs_horizon, :]
        return nsample

    def normalize_obs(self, data):
        return normalize_data(data, self.stats['obs'])

    def unnormalize_obs(self, data):
        return unnormalize_data(data, self.stats['obs'])

    def normalize_action(self, data):
        return normalize_data(data,
                              self.stats['action'],
                              center=not self.action_delta)

    def unnormalize_action(self, data):
        return unnormalize_data(data,
                                self.stats['action'],
                                center=not self.action_delta)


# dataset
class GmlDataset(torch.utils.data.Dataset):
    def __init__(self,
                 dataset_path,
                 sequence_length,
                 pad_before=0,
                 pad_after=0,
                 stride=1,
                 action_delta=False,
                 action_penlift=False,
                 normalize=defaultdict(lambda: True),
                 max_drawings: Optional[int] = None):

        # read from zarr dataset
        dataset_root = zarr.open(dataset_path, 'r')
        # All demonstration episodes are concatinated in the first dimension N
        action_key = 'action' if action_delta else 'state'
        train_data = {
            # (N, action_dim)
            'action': dataset_root['data'][action_key][:].astype(np.float32),
            # (N, obs_dim)
            'obs': dataset_root['data']['state'][:].astype(np.float32)
        }
        print(train_data['action'].shape, train_data['obs'].shape)
        if action_penlift:
            dx_act = np.diff(train_data['obs'], axis=0, append=-8675309)
            dx_pred = dataset_root['data']['action'][:].astype(np.float32)
            if dx_pred.shape[1] == 3:
                # tmp = dx_pred[:, 2]
                # dx_pred = dx_pred[:, :2]
                # train_data['action'][:, 2] = train_data['action'][:, 2] < 0.5
                pass
            else:
                pen_lifted = np.linalg.norm(dx_act - dx_pred, axis=1) > 1e-6
                # print(np.stack((tmp[1800:1900] < 0.5, pen_lifted[1800:1900])).T)
                # print(np.argwhere((tmp < 0.5) != pen_lifted).flatten()[:10])
                # np.testing.assert_allclose(tmp < 0.5, pen_lifted)
                if action_delta:
                    train_data['action'][pen_lifted] = dx_act[pen_lifted]
                    train_data['action'][-1] = 0
                train_data['action'] = np.concatenate(
                    [train_data['action'], pen_lifted[:, None]], axis=1)

        # Marks one-past the last index for each episode
        if max_drawings is None:
            episode_ends = dataset_root['meta']['episode_ends'][:]
        else:
            episode_ends = dataset_root['meta']['episode_ends'][:max_drawings]
            train_data['action'] = train_data['action'][:episode_ends[-1] + 10]
            train_data['obs'] = train_data['obs'][:episode_ends[-1] + 10]

        # compute start and end of each state-action sequence
        # also handles padding
        indices = GmlDataset.create_sample_indices(
            episode_ends=episode_ends,
            sequence_length=sequence_length,
            # add padding such that each timestep in the dataset are seen
            pad_before=pad_before,
            pad_after=pad_after,
            stride=stride)

        # compute statistics and normalized data to [-1,1]
        stats = dict()
        normalized_train_data = dict()
        for key, data in train_data.items():
            if normalize[key]:
                stats[key] = get_data_stats(data)
                if key == 'action' and action_penlift and action_delta:
                    lifted = data[:, -1]
                    stats[key] = get_data_stats(data[lifted == 0])
                    stats[key]['min'][-1] = 0
                    stats[key]['max'][-1] = 10
                normalized_train_data[key] = normalize_data(data, stats[key])
            else:
                normalized_train_data[key] = data

        self.indices = indices
        self.stats = stats
        self.normalized_train_data = normalized_train_data
        self.normalize = normalize
        self.episode_ends = episode_ends
        self.sequence_length = sequence_length
        self.pad_after = pad_after
        self.pad_before = pad_before
        self.action_delta = action_delta
        self.action_penlift = action_penlift

    @staticmethod
    def create_sample_indices(episode_ends: np.ndarray,
                              sequence_length: int,
                              pad_before: int = 0,
                              pad_after: int = 0,
                              stride: int = 1):
        indices = list()
        for i in range(len(episode_ends)):
            start_idx = 0
            if i > 0:
                start_idx = episode_ends[i - 1]
            end_idx = episode_ends[i]
            episode_length = end_idx - start_idx

            # range stops one idx before end
            # First do pre-buffer
            buffer_start_idx = np.zeros(pad_before, dtype=int)
            buffer_end_idx = sequence_length - 1 - np.arange(pad_before,
                                                             dtype=int)
            sample_start_idx = np.arange(pad_before, dtype=int) + 1
            sample_end_idx = sequence_length + np.zeros(pad_before, dtype=int)
            indices.extend(
                np.stack([
                    buffer_start_idx + start_idx, buffer_end_idx + start_idx,
                    sample_start_idx, sample_end_idx
                ],
                         axis=1)[::stride])
            # Next do main buffer
            if episode_length >= sequence_length:
                buffer_start_idx = np.arange(episode_length - sequence_length +
                                             1,
                                             dtype=int)
                buffer_end_idx = buffer_start_idx + sequence_length
                sample_start_idx = np.zeros(episode_length - sequence_length +
                                            1,
                                            dtype=int)
                sample_end_idx = sequence_length + np.zeros(
                    episode_length - sequence_length + 1, dtype=int)
                indices.extend(
                    np.stack([
                        buffer_start_idx + start_idx, buffer_end_idx +
                        start_idx, sample_start_idx, sample_end_idx
                    ],
                             axis=1)[::stride])
            # Finally do post-buffer
            buffer_start_idx = episode_length - sequence_length + 1 + np.arange(
                pad_after, dtype=int)
            buffer_end_idx = episode_length + np.zeros(pad_after, dtype=int)
            sample_start_idx = np.zeros(pad_after, dtype=int)
            sample_end_idx = sequence_length - 1 - np.arange(pad_after,
                                                             dtype=int)
            indices.extend(
                np.stack([
                    buffer_start_idx + start_idx, buffer_end_idx + start_idx,
                    sample_start_idx, sample_end_idx
                ],
                         axis=1)[::stride])

        indices = np.array(indices)
        return indices

    def __len__(self):
        # all possible segments of the dataset
        return len(self.indices)

    def __getitem__(self, idx):
        # get the start/end indices for this datapoint
        buffer_start_idx, buffer_end_idx, \
            sample_start_idx, sample_end_idx = self.indices[idx]

        # get nomralized data using these indices
        try:
            nsample = sample_sequence(train_data=self.normalized_train_data,
                                      sequence_length=self.sequence_length,
                                      buffer_start_idx=buffer_start_idx,
                                      buffer_end_idx=buffer_end_idx,
                                      sample_start_idx=sample_start_idx,
                                      sample_end_idx=sample_end_idx,
                                      action_delta=self.action_delta)
        except:
            print(buffer_start_idx, buffer_end_idx, sample_start_idx,
                  sample_end_idx)
            raise

        # discard unused observations
        # nsample['obs'] = nsample['obs'][:self.obs_horizon,:]
        assert nsample['obs'].shape[0] == self.sequence_length
        return nsample

    def normalize_obs(self, data):
        return normalize_data(
            data, self.stats['obs']) if self.normalize['obs'] else data

    def unnormalize_obs(self, data):
        return unnormalize_data(
            data, self.stats['obs']) if self.normalize['obs'] else data

    def normalize_action(self, data):
        return normalize_data(
            data, self.stats['action'],
            center=not self.action_delta) if self.normalize['action'] else data

    def unnormalize_action(self, data):
        return unnormalize_data(
            data, self.stats['action'],
            center=not self.action_delta) if self.normalize['action'] else data

    def create_normalized_from_drawing(self,
                                       drawing: Drawing,
                                       strokei: Optional[int] = None):
        def compute_state(stroke):
            return stroke[:, 1:3].astype(np.float32)

        def compute_action_(stroke):
            return np.diff(stroke[:, 1:3], axis=0,
                           append=stroke[-1:, 1:3]).astype(np.float32)

        compute_action = compute_action_ if self.action_delta else compute_state

        if strokei is not None:
            obs = compute_state(drawing.strokes[strokei])
            act = compute_action(drawing.strokes[strokei])
        else:
            if not self.action_penlift:
                obs = np.concatenate(
                    [compute_state(stroke) for stroke in drawing.strokes])
                act = np.concatenate(
                    [compute_action(stroke) for stroke in drawing.strokes])
            else:

                def append_false(stroke):
                    return np.concatenate(
                        [stroke, np.zeros((stroke.shape[0], 1), dtype=np.float32)], axis=1)

                all_obs = [compute_state(drawing.strokes[0])]
                all_act = [append_false(compute_action(drawing.strokes[0]))]
                for strokei in range(1, len(drawing.strokes)):
                    obs = compute_state(drawing.strokes[strokei])
                    act = append_false(compute_action(
                        drawing.strokes[strokei]))
                    all_act[-1][-1] = [*(obs[0] - all_obs[-1][-1]), 1]
                    all_obs.append(obs)
                    all_act.append(act)
                obs = np.concatenate(all_obs)
                act = np.concatenate(all_act)

        return self.normalize_obs(obs), self.normalize_action(act)
