import numpy as np
from numpy.core.function_base import linspace

def capture_alphabet(data_file):
        """Read from mocap data

        Args:
            data_file ([.npz]): [a bunch of .npy files]
        """
        # currently using 'all_letters.npz'
        data = np.load(data_file, allow_pickle=True)
        return(data)

def parse_trajectory(input_sequence):
        """Parse trajectory from mocap data

        Args:
            alphabet ([.npz]): [a bunch of .npy files]
        """
        data = capture_alphabet("all_letters.npz")
        print(data)
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
                t_step = np.empty(len(strokea1))
                for i in range(len(t_step)):
                    t_step[i] = 1/120*i
                traj.append(np.array([t_step, strokea1[:,0], strokea1[:,1]]))
            if input == "B":
                t_step = np.empty(len(strokeb1))
                for i in range(len(t_step)):
                    t_step[i] = 1/120*i
                traj.append(np.array([t_step, strokeb1[:,0], strokeb1[:,1]]))
            if input == "C":
                t_step = np.empty(len(strokec1))
                for i in range(len(t_step)):
                    t_step[i] = 1/120*i
                traj.append(np.array([t_step, strokec1[:,0], strokec1[:,1]]))
            if input == "D":
                t_step = np.empty(len(stroked1))
                for i in range(len(t_step)):
                    t_step[i] = 1/120*i
                traj.append(np.array([t_step, stroked1[:,0], stroked1[:,1]]))
            if input == "E":
                t_step = np.empty(len(strokee1))
                for i in range(len(t_step)):
                    t_step[i] = 1/120*i
                traj.append(np.array([t_step, strokee1[:,0], strokee1[:,1]]))
            if input == "F":
                t_step = np.empty(len(strokef1))
                for i in range(len(t_step)):
                    t_step[i] = 1/120*i
                traj.append(np.array([t_step, strokef1[:,0], strokef1[:,1]]))
            if input == "G":
                traj.append(np.array([strokeg1]))
            if input == "Test":  # plot basic test case
                traj = np.array([[strokea1, strokea2],
                                [strokeb1, strokeb2, strokeb3],
                                [strokec1], [stroked1, stroked2]])
        return(traj)