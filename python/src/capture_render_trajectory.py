import numpy as np
from numpy.core.function_base import linspace

# from art_skills.chebyshev_fitter import ChebyshevFit

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

        data_b = data['B']
        strokeb1 = data_b[1]
        strokeb2 = data_b[3]
        strokeb3 = data_b[5]

        data_c = data['C']
        strokec1 = data_c[1]

        data_d = data['D']
        stroked1 = data_d[1]
        stroked2 = data_d[3]

        data_e = data['E']
        strokee1 = data_e[1]

        data_f = data['F']
        strokef1 = data_f[1]

        data_g = data['G']
        strokeg1 = data_g[1]

        traj = []
        for input in input_sequence:
            if input == "A":
                traj.append(np.array([strokea1, strokea2]))
            if input == "B":
                traj.append(np.array([strokeb1, strokeb2, strokeb3]))
            if input == "C":
                traj.append(np.array([strokec1]))
            if input == "D":
                traj.append(np.array([stroked1, stroked2]))
            if input == "E":
                traj.append(np.array([strokee1]))
            if input == "F":
                traj.append(np.array([strokef1]))
            if input == "G":
                traj.append(np.array([strokeg1]))
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
                interp_x.append(ChebyshevFit(data_x, traj_t, order))
                interp_y.append(ChebyshevFit(data_y, traj_t, order))
                count_s += 1
            count_l += 1
        return(interp_x, interp_y)