"""
io.py
IO functions for loading and saving data.
@author Gerry
"""
import pandas as pd

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
