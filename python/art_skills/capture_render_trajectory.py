import numpy as np
from numpy.core.function_base import linspace
# from matplotlib import pyplot as plt


class CaptureRenderTrajectory:
    """A class for generating the trajectory from data"""

    def capture_alphabet(self, data_file):
        """Read from mocap data

        Args:
            data_file ([.npz]): [a bunch of .npy files]
        """
        # currently using 'all_letters.npz'
        data = np.load(data_file, allow_pickle=True)
        return(data)

    def parse_trajectory(self, input_sequence):
        """Parse trajectory from mocap data

        Args:
            alphabet ([.npz]): [a bunch of .npy files]
        """
        data = self.capture_alphabet("all_letters.npz")
        traj = []
        # data_a is a list of strokes, where each stroke is an Nx2 numpy array
        data_a = data['A']
        strokea1 = data_a[1]
        strokea2 = data_a[3]
        # plt.plot(strokea1[:, 0], strokea1[:, 1])
        # plt.plot(strokea2[:, 0], strokea2[:, 1])
        # plt.show()

        data_b = data['B']
        strokeb1 = data_b[1]
        strokeb2 = data_b[3]
        strokeb3 = data_b[5]

        data_c = data['C']
        strokec1 = data_c[1]

        data_d = data['D']
        stroked1 = data_d[1]
        stroked2 = data_d[3]

        traj = []
        for input in input_sequence:
            if input == "A":
                traj.append(np.array([strokea1, strokea2]))
            if input == "B":
                traj.append(np.array([strokeb1, strokeb2, strokeb3]))
            if input == "Test":  # plot basic test case
                traj = np.array([[strokea1, strokea2],
                                [strokeb1, strokeb2, strokeb3],
                                [strokec1], [stroked1, stroked2]])
        return(traj)

    def render_trajectory(self, traj, order):
        """
        Render trajectory that reproduces artistic intent.

        Args:
            traj ([array]): [the letters and strokes of each letter]
            order ([int]): [the order of chebyshev polynomial]
        """
        interp_x = []
        interp_y = []
        count_s = 0
        count_l = 0
        for letter in traj:
            for stroke in letter:
                traj_x = (stroke[:, 0] + count_l*0.7)
                traj_y = (stroke[:, 1])
                traj_t = linspace(0, 1, len(traj_x))
                zipx = zip(traj_t, traj_x)
                zipy = zip(traj_t, traj_y)
                data_x = dict(zipx)
                data_y = dict(zipy)
                interp_x.append(self.chebyshev_fit(data_x, traj_t, order))
                interp_y.append(self.chebyshev_fit(data_y, traj_t, order))
                count_s += 1
            count_l += 1
        return(interp_x, interp_y)