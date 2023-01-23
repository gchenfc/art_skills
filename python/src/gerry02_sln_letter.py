# Autogenerated, and modified, from gerry02_sln_letter.ipynb
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from IPython.display import HTML
import tqdm
import sln_letter_fit
from sln_letter_fit import FitParams, OptimizationLoggingParams
import loader, plotting

artist = 'max'
fname_format = 'result/2022-04-26/2stage_' + artist + '_{:}'

for letter in 'ABCDEFGHIJKLMNOPQRSTUVWXYZ':
    fbase = fname_format.format(letter)
    fig, ax = plt.subplots(figsize=(5, 4))
    sols_and_histories, anim = sln_letter_fit.fit_and_plot_trajectories(
        ax,
        letter,
        artist=artist,
        num_strokes=None,
        # trajectory_indices=(0,),
        trajectory_indices=None,
        max_iters=1000,
        fit_params_kwargs={
            'reparameterize': False,
            'flip_parameters_at_end': True,
            'initialization_strategy_params': ' :D '
        },
        log_history=True,
        animate=True,
        save_animation_fname=fbase + '.mp4',
        animation_oversample=1,
        use_2_stage=True)
    plt.close(fig)

    np.save(fbase, np.array(sols_and_histories, dtype=object), allow_pickle=True)


# In[ ]:


# b = np.load('data/optimized_A.npy', allow_pickle=True)
# traj1 = b[0]
# sol1, history1 = traj1
# print(sol1.keys())
# print(history1[0]['params'])
# print(history1[0]['txy'].shape)
# print(history1[0]['txy_from_params'].shape)
# print(history1[0]['stroke_indices'])


# # Combine all letters into a single Video

# In[ ]:


# Combine all letters into a single video
import cv2

w_, h_ = 500, 400
w, h = w_ * 7, h_ * 4
T = 101
imgs = np.ones((T, h, w, 3), dtype=np.uint8) * 255

for i, letter in enumerate(tqdm.tqdm('ABCDEFGHIJKLMNOPQRSTUVWXYZ')):
    xi, yi = i % 7, i // 7
    x, y = xi * w_, yi * h_
    vid = cv2.VideoCapture(fname_format.format(letter) + '.mp4')
    for k in range(T):
        success, frame = vid.read()
        if success:
            imgs[k, y:y + h_, x:x + w_, :] = frame

fps = 30
out = cv2.VideoWriter(
    fname_format.format('_all') + '.mp4', cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h), True)
for frame in imgs:
    out.write(frame)
out.release()
out = None
