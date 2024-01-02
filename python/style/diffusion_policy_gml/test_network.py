import unittest

import torch
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler
from network import compute_orig, compute_noise, MemorizationModel, ConditionalUnet1D


noise_scheduler = DDPMScheduler(
    num_train_timesteps=76,
    # the choise of beta schedule has big impact on performance
    # we found squared cosine works the best
    beta_schedule='squaredcos_cap_v2',
    # clip output to [-1,1] to improve stability
    clip_sample=True,
    clip_sample_range=5,
    # our network predicts noise (instead of denoised action)
    prediction_type='epsilon'
)

orig = torch.randn(10, 2, 32)
noise = torch.randn(10, 2, 32)
timesteps = torch.randint(76, (10,))


class TestNetwork(unittest.TestCase):
    def test_compute_noise_and_orig(self):
        """Tests the compute_noise and compute_orig methods."""
        noisy = noise_scheduler.add_noise(orig, noise, timesteps)
        orig_pred = compute_orig(noise_scheduler, timesteps, noisy, noise)
        noise_pred = compute_noise(noise_scheduler, timesteps, noisy, orig)

        torch.testing.assert_close(orig_pred, orig)
        torch.testing.assert_close(noise_pred, noise)


if __name__ == '__main__':
    unittest.main()
