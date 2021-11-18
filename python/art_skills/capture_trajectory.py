import numpy as np
# from matplotlib import pyplot as plt


class CaptureTrajectory:
    """A class for generating the trajectory from data"""

    def capture_alphabet(data_file):
        """Read from mocap data

        Args:
            data_file ([.npz]): [a bunch of .npy files]
        """
        # currently using 'all_letters.npz'
        data = np.load(data_file, allow_pickle=True)
        return(data)

    def capture_trajectory(self, input_sequence):
        """Parse

        Args:
            alphabet ([.npz]): [a bunch of .npy files]
        """
        data = np.load('all_letters.npz', allow_pickle=True)
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
