"""
#@markdown ### **Environment**
#@markdown Defines a Drawing Simulation
"""

import numpy as np
import cv2

# env
class PaintingEnv():
    metadata = {"render.modes": ["human", "rgb_array"], "video.frames_per_second": 10}
    reward_range = (0., 1.)

    def __init__(self, resolution=(128, 128)):
        self.resolution = resolution
        self.reset()

    def reset(self, x0=[0, 0]):
        self.pos = np.array(x0, dtype=np.float32)
        self.traj = [self.pos]

        # return obs, info
        return self.traj[-1], None

    def step(self, action):
        self.pos = self.pos + action
        self.traj.append(self.pos)

        # return observation, reward, terminated, truncated, info
        return self.traj[-1], 0, False, False, None

    def render(self, mode):
        im = np.zeros((*self.resolution, 3), dtype=np.uint8)

        traj = self._normalize(np.array(self.traj)).astype(int)
        x1, y1 = traj[0]
        for x2, y2 in traj[1:]:
            cv2.line(im, (x1, y1), (x2, y2), (255, 255, 255), 1)
            x1, y1 = x2, y2

        return im

    def _normalize(self, traj):
        # xmin, xmax = stats['obs']['min'][0], stats['obs']['max'][0]
        # ymin, ymax = stats['obs']['min'][1], stats['obs']['max'][1]
        # # [ -30.353271 -275.40967 ]
        # # [497.96973  29.76587]
        # xmin, xmax = -500, 500
        # ymin, ymax = -500, 500
        xmin, xmax = max(-500, np.min(traj[:, 0])), min(500, np.max(traj[:, 0]))
        ymin, ymax = max(-500, np.min(traj[:, 1])), min(500, np.max(traj[:, 1]))
        return (traj - np.array([xmin, ymin])) / np.array([xmax-xmin, ymax-ymin]) * np.array(self.resolution)
