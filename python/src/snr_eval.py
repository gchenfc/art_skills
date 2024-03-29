import sys
import numpy as np
import pickle
import matplotlib.pyplot as plt
import scipy.signal
import scipy.integrate


def speed(data):
    v = np.diff(data[:, 1:], axis=0) / np.diff(data[:, 0]).reshape(-1, 1)
    speed2 = np.sum(np.square(v), axis=1)
    try:
        speed2_smooth = scipy.signal.savgol_filter(speed2, 5, 1)
    except ValueError:
        print('Warning: Trajectory too short to smooth speed for local minima checking: ', speed2)
        speed2_smooth = speed2
    return speed2


def SNR(input_data, output_data):
    numerator = scipy.integrate.simpson(input_data**2)
    denominator = scipy.integrate.simpson((input_data - output_data)**2)
    return 10 * np.log10(numerator / denominator)
    # return 10*np.log(scipy.integrate.simpson(input_data**2) /
    #           scipy.integrate.simpson((input_data-output_data)**2))


##################################################################################


filename = sys.argv[1]  # read filename from command line
with open(filename, 'rb') as f:
    snr_history = pickle.load(f)

filename2 = sys.argv[2]  # read filename from command line
with open(filename2, 'rb') as f2:
    full_history = pickle.load(f2)

params = full_history[-1]
print(params)

# history: n rows of [input_stroke, estimated_output], where n is # of points
# input_stroke: np.array(t, x, y) with n(i) rows (n at that iteration)
# estimated_output: np.array(t, x, y)

smooth_acc = snr_history[-1][2]
inflections = snr_history[-1][3]

input_stroke = snr_history[-1][0]
estimated_output = snr_history[-1][1]
output_stroke = []
for _, new_output, _, _ in snr_history:
    output_stroke.extend(new_output[len(output_stroke):])
output_stroke = np.array(output_stroke)
# print(input_stroke.shape, estimated_output.shape, output_stroke.shape)

# print("___________________________________ Input")
# print(input_stroke)
# print("___________________________________ Estimated")
# print(estimated_output)
# print("___________________________________ Output")
# print(output_stroke)

in_speed = speed(input_stroke)
est_speed = speed(estimated_output)
out_speed = speed(output_stroke)

est_SNR = SNR(in_speed, est_speed)
out_SNR = SNR(in_speed, out_speed)

plt.figure(1)
plt.plot(input_stroke[:-1, 0], in_speed, 'k-',
         linewidth=1, label='motion input')
plt.plot(estimated_output[:-1, 0],
         est_speed,
         'b-',
         linewidth=1,
         label='final estimate (SNR: %s)' % float('%.3g' % est_SNR))
plt.plot(output_stroke[:-1, 0],
         out_speed,
         'g-',
         linewidth=1,
         label='online output: (SNR: %s)' % float('%.3g' % out_SNR))
# plt.plot(input_stroke[:-1, 0], 1 / np.diff(input_stroke[:, 0]), 'r-', linewidth=1, label='dt output: (SNR: %s)' % float('%.3g' % out_SNR))
# plt.ylim(0, 7)
plt.legend()
#output_stroke[:-4, 0], 
print("INFLECTIONS\n",inflections, type(inflections))
plt.figure(2)
plt.plot(smooth_acc, 'g-', linewidth=1, label='smoothed accel')
plt.plot(inflections, smooth_acc[inflections], 'k*', label='inflections')
plt.show()
