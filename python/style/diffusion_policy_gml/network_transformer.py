from typing import Union, Optional
import torch
import torch.nn as nn
from style.diffusion_policy_gml.transformer_for_diffusion import TransformerForDiffusion
from diffusion_policy.model.diffusion.positional_embedding import SinusoidalPosEmb
import json

# class AugmentWithCnn(nn.Module):

#     def __init__(self, input_dim, n_emb):
#         super().__init__()
#         self.layer1 = nn.Conv1d(input_dim, input_dim * 3, 5)
#         self.layer2 = nn.Conv1d(input_dim * 3, n_emb, 1)
#         self.input_emb = nn.Sequential(nn.Conv1d(input_dim, input_dim * 3, 5),
#                                        nn.ReLU(), nn.Conv1d(n_emb, n_emb, 1),
#                                        nn.ReLU(), nn.Conv1d(n_emb, n_emb, 1),
#                                        nn.ReLU())

#     def forward(self, x):
#         return self.input_emb(x)


class Transpose(nn.Module):

    def __init__(self, dim1, dim2):
        super().__init__()
        self.dim1 = dim1
        self.dim2 = dim2

    def forward(self, x):
        return x.transpose(self.dim1, self.dim2)


class Transformer1d(TransformerForDiffusion):

    def __init__(
        self,
        input_dim: int,
        output_dim: int,
        horizon: int,
        n_obs_steps: int = None,
        cond_dim: int = 0,
        n_layer: int = 12,
        n_head: int = 12,
        n_emb: int = 768,
        p_drop_emb: float = 0.1,
        p_drop_attn: float = 0.1,
        causal_attn: bool = False,
        time_as_cond: bool = True,
        obs_as_cond: bool = False,
        n_cond_layers: int = 0,
        use_sinusoidal_pos_embedding: bool = False,
        augment_input_embedding: Optional[nn.Module] = None,
    ) -> None:
        self.use_sin_pos_emb = use_sinusoidal_pos_embedding
        super().__init__(input_dim=input_dim,
                         output_dim=output_dim,
                         horizon=horizon,
                         n_obs_steps=n_obs_steps,
                         cond_dim=cond_dim,
                         n_layer=n_layer,
                         n_head=n_head,
                         n_emb=n_emb,
                         p_drop_emb=p_drop_emb,
                         p_drop_attn=p_drop_attn,
                         causal_attn=causal_attn,
                         time_as_cond=time_as_cond,
                         obs_as_cond=obs_as_cond,
                         n_cond_layers=n_cond_layers)
        # if use_sinusoidal_input_embedding:
        #     self.input_emb = nn.Linear(input_dim, n_emb)
        # initialize positional embedding to sinusoidal
        if augment_input_embedding is not None:
            # augment with cnn and stuff
            self.input_emb = augment_input_embedding
        self.apply(self._init_weights)

    def _init_weights(self, module):
        if isinstance(module, TransformerForDiffusion):
            if self.use_sin_pos_emb:
                _, T, n_emb = module.pos_emb.shape
                tmp = SinusoidalPosEmb(n_emb)
                # module.pos_emb = nn.Parameter(
                #     tmp(torch.arange(T))[None, ...].to(module.pos_emb.device))
                pos_emb = module.pos_emb
                # module._buffers.pop(
                #     'pos_emb', None)  # If pos_emb is registered as parameter
                module._parameters.pop(
                    'pos_emb', None)  # If pos_emb is registered as parameter
                pos_emb = tmp(torch.arange(T))[None, ...].to(pos_emb.device)
                # pos_emb[:, :, 20:64] = 0
                # pos_emb[:, :, 64 + 20:] = 0
                pos_emb.requires_grad = False
                module.register_buffer('pos_emb', pos_emb)
            else:
                torch.nn.init.normal_(module.pos_emb, mean=0.0, std=0.02)
            if module.cond_obs_emb is not None:
                torch.nn.init.normal_(module.cond_pos_emb, mean=0.0, std=0.02)
        elif isinstance(module, nn.Conv1d):
            torch.nn.init.kaiming_normal_(module.weight)
            if module.bias is not None:
                torch.nn.init.constant_(module.bias, 0)
        elif isinstance(module, nn.ReLU) or isinstance(module, Transpose):
            pass
        else:
            super()._init_weights(module)

    @staticmethod
    def FromJson(fname, augment_input_embedding=None):
        """Construct from a json file.
        Because augment_input_embedding gets serialized as a string, we need to get it passed in.
        """
        with open(fname) as f:
            network_kwargs = json.load(f)
        assert str(augment_input_embedding) == network_kwargs['augment_input_embedding'], \
            f"{str(augment_input_embedding)} != {network_kwargs['augment_input_embedding']}"
        network_kwargs['augment_input_embedding'] = augment_input_embedding
        return Transformer1d(**network_kwargs), network_kwargs

    def forward(self,
                sample: torch.Tensor,
                timestep: Union[torch.Tensor, float, int],
                global_cond=None):
        """
        sample: (B,T,input_dim)
        timestep: (B,) or int, diffusion step
        global_cond: (B,global_cond_dim)
        output: (B,T,input_dim)
        """

        # 1. time
        timesteps = timestep
        if not torch.is_tensor(timesteps):
            # TODO: this requires sync between CPU and GPU. So try to pass timesteps as tensors if you can
            timesteps = torch.tensor([timesteps],
                                     dtype=torch.long,
                                     device=sample.device)
        elif torch.is_tensor(timesteps) and len(timesteps.shape) == 0:
            timesteps = timesteps[None].to(sample.device)
        # broadcast to batch dimension in a way that's compatible with ONNX/Core ML
        timesteps = timesteps.expand(sample.shape[0])
        time_emb = self.time_emb(timesteps).unsqueeze(1)
        # (B,1,n_emb)

        # process input
        input_emb = self.input_emb(sample)  # (B, T, n_emb)

        if global_cond is not None:
            raise NotImplementedError("Global condition not supported")

        if self.encoder_only:
            # BERT
            token_embeddings = torch.cat([time_emb, input_emb], dim=1)
            t = token_embeddings.shape[1]
            position_embeddings = self.pos_emb[:, :t, :]  # (learnable) vectors
            x = self.drop(token_embeddings + position_embeddings)
            # (B,T+1,n_emb)
            x = self.encoder(src=x, mask=self.mask)
            # (B,T+1,n_emb)
            x = x[:, 1:, :]
            # (B,T,n_emb)
        else:
            # encoder
            cond_embeddings = time_emb
            if self.obs_as_cond:
                cond_obs_emb = self.cond_obs_emb(cond)
                # (B,To,n_emb)
                cond_embeddings = torch.cat([cond_embeddings, cond_obs_emb],
                                            dim=1)
            tc = cond_embeddings.shape[1]
            position_embeddings = self.cond_pos_emb[:, :tc, :]  # learnable
            x = self.drop(cond_embeddings + position_embeddings)
            x = self.encoder(x)
            memory = x
            # (B,T_cond,n_emb)

            # decoder
            token_embeddings = input_emb
            t = token_embeddings.shape[1]
            position_embeddings = self.pos_emb[:, :
                                               t, :]  # each position maps to a (learnable) vector
            x = self.drop(token_embeddings + position_embeddings)
            # (B,T,n_emb)
            x = self.decoder(tgt=x,
                             memory=memory,
                             tgt_mask=self.mask,
                             memory_mask=self.memory_mask)
            # (B,T,n_emb)

        # head
        x = self.ln_f(x)
        x = self.head(x)
        # (B,T,n_out)
        return x
