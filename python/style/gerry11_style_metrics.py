# %%
import cv2
import torch
import tqdm.auto as tqdm
import matplotlib.pyplot as plt
from pathlib import Path
import torch_fidelity

from gerry11_all_networks import (Network, Base, Decoupled, Guidance,
                                  Finetuned, Control, ClassifierFree1)
from style.diffusion_policy_gml.dataset import GmlDatasetNoSliding

# %%
PRED_HORIZON = 128
DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'

# %% [markdown]
# ## GML Images
# These will actually not work because they have backgrounds and styles and stuff.

# %%
if False:
    gml_images_folder = Path("data/gml_images")
    display(sorted(gml_images_folder.iterdir())[-10:])
    pic_fnames = gml_images_folder.glob('[0-9]*.jpg')
    pic_numbers = sorted([int(pic.stem) for pic in pic_fnames])
    print(pic_numbers[:10])
    print(pic_numbers[-10:])

    example_img = cv2.imread((gml_images_folder / "69815.jpg").as_posix())
    print(example_img.shape, example_img.dtype)
    plt.imshow(example_img)

# %% [markdown]
# ## GML dataset

# %%
dataset = GmlDatasetNoSliding(
    dataset_path=Path(
        'data/gml_by_drawing_PRESERVE_ASPECT_CENTERED_003000.zarr'),
    sequence_length=PRED_HORIZON,
    action_delta=True,
    action_penlift=True,
    ignore_jump_actions=True,
    normalize=dict(obs=False, action=True),
    # max_drawings=5,
    # min_traj_length=15
)

# %%
for sample in dataset:
    print(sample['obs'].shape)
    print(sample['obs'].min(), sample['obs'].max())
    example_img = Network.render_img(sample['obs'])
    plt.imshow(example_img)
    break

# %%
fig, axes = plt.subplots(4, 4, figsize=(16, 16))
for ax, sample in zip(axes.flatten(), dataset):
    ax.imshow(Network.render_img(sample['obs']))
    ax.axis('off')


# %%
class GmlImagesDataset(torch.utils.data.Dataset):

    def __init__(
        self,
        dataset_path=Path(
            'data/gml_by_drawing_PRESERVE_ASPECT_CENTERED_003000.zarr'),
        sequence_length=PRED_HORIZON,
        action_delta=True,
        action_penlift=True,
        ignore_jump_actions=True,
        normalize=dict(obs=False, action=True)):
        self.dataset = GmlDatasetNoSliding(
            dataset_path=dataset_path,
            sequence_length=sequence_length,
            action_delta=action_delta,
            action_penlift=action_penlift,
            ignore_jump_actions=ignore_jump_actions,
            normalize=normalize,
        )

    def __getitem__(self, idx):
        return torch.from_numpy(Network.render_img(self.dataset[idx]['obs']))

    def __len__(self):
        return len(self.dataset)


gml_images_dataset = GmlImagesDataset()
fig, axes = plt.subplots(4, 4, figsize=(16, 16))
for ax, img in zip(axes.flatten(), gml_images_dataset):
    ax.imshow(img.cpu().numpy())
    ax.axis('off')
# %%
folder = Path('data/gml_images_2/')
folder.mkdir(exist_ok=True)
for i, img in enumerate(tqdm.tqdm(gml_images_dataset)):
    cv2.imwrite((folder / f'{i:05d}.png').as_posix(), img.cpu().numpy())

# %%
torch_fidelity.register_dataset('gml-ds',
                                lambda root, download: gml_images_dataset)

# %% [markdown]
# ## Model outputs
base = Base()

# %%
imgs = base.generate_img(16, PRED_HORIZON)

fig, axes = plt.subplots(4, 4, figsize=(16, 16))
for ax, sample in zip(axes.flatten(), imgs):
    ax.imshow(sample)
    ax.axis('off')

# %%
torch.manual_seed(8675309)
imgs = base.generate_img(100, PRED_HORIZON)
folder = Path('results/gerry11_all_networks/Base')
for i, img in enumerate(imgs):
    cv2.imwrite((folder / f'Base_{i:02d}.png').as_posix(), img)

# %% [markdown]
#


class Debug(torch.nn.Module):

    def forward(noise):
        print(noise.shape)


wrapped_generator = torch_fidelity.GenerativeModelModuleWrapper(
    # lambda noise: base.generate_img(noise.shape[0], noise.shape[1]),
    Debug(),
    128,
    'normal',
    0)

with torch.no_grad():
    metrics_dict = torch_fidelity.calculate_metrics(
        input1='gml-ds',
        input2='gml-ds',
        cuda=True,
        isc=True,
        fid=True,
        kid=True,
        verbose=False,
    )

# %%
