"""
io.py
IO functions for loading and saving data.
@author Gerry
"""
import pandas as pd
from .communicate import CableRobot
import time

def load_log(fname):
    data = pd.read_csv('logs/G.txt', header=None, names=['t', 'c', 't1', 'x', 'y'])

    # Split into strokes
    stroke_starts = data[data['c'] == 'M'].index
    stroke_ends = data[data['c'] == 'U'].index
    strokes = [data[s:e + 1][['t1', 'x', 'y']].to_numpy() for s, e in zip(stroke_starts, stroke_ends)]
    for stroke in strokes:
        stroke[:, 2] *= -1  # flip y-axis
        stroke[:, 0] -= strokes[0][0, 0]
    return strokes

def export_spline_serial(spline):
    """Writes a piecewise polynomial path to a c++ header file

    Args:
        spline (scipy.PPoly): Object with properties `c` (degree+1, segments, xy) representing the
                              coefficients and `x` (segments+1) representing the knots
    """
    deg, n_seg, xy = spline.c.shape
    assert xy == 2, 'Expected a 2D spline, but either was not 2D or was of unexpected form'
    assert deg - 1 == 3, 'Expected a cubic spline, but either was not cubic or was of unexpected form'
    assert spline.x[0] == 0, 'Expected the spline to start at t = 0.'
    assert len(spline.x) == n_seg + 1, 'Expected the spline to have n_seg + 1 boundary points.'

    for i in range(n_seg):
        t, x = spline.x[i + 1], spline.c[:, i, :].squeeze()
        yield ','.join(map(str, [t, *x.T.flatten()]))

def send_spline_serial(spline, port='/dev/tty.usbmodem100994303'):
    import time
    with CableRobot(print_raw=True, write_timeout=None, initial_msg='d10,100', port=port, silent=True) as robot:
        robot.update()

        robot.send('gs1')
        robot.send('t-')
        for line in export_spline_serial(spline):
            robot.send('t&' + line)

        robot.update()

        # Read-back checks
        robot.send('t#')
        act_num = read_spline_serial(robot, num_lines=1)[0].split(':')[-1][1:]
        assert act_num == str(
            spline.c.shape[1]), f'Robot only read {act_num} / {spline.c.shape[1]} segments'
        print('Read-back check passed')

        # Other checks we could implement later if desired
        # robot.send('t*')
        # read_spline_serial(robot, num_lines=10)
        # robot.send('t>1.5')
        # read_spline_serial(robot, num_lines=1)
        # robot.send('t<10')
        # read_spline_serial(robot, num_lines=1)

def read_spline_serial(robot: CableRobot, timeout=2, num_lines=10):
    s = ''
    all_lines = []
    tstart = time.time()
    while (time.time() - tstart) < timeout:
        s += robot.read()
        lines = s.split('\n')
        lines, s = lines[:-1], lines[-1]
        lines = [line for line in lines if line.startswith('Spline ') or line.startswith('Debug Spline')]
        all_lines.extend(lines)
        if len(all_lines) >= num_lines:
            break
    return all_lines
