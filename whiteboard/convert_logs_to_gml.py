from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import pickle
import argparse
import tqdm.auto as tqdm

# in the python directory, `pip install -e .`
from style import load_gml
from gerry import Stopwatch

# these should be relative to this directory
THIS_FOLDER = Path(__file__).parent
DEFAULT_INFOLDER = THIS_FOLDER / 'logs/'
DEFAULT_OUTFOLDER = THIS_FOLDER / '../python/style/data/log_gmls/'


def to_txy(log):
    codes = ['M', 'L', 'U']
    tptxy = np.loadtxt(log,
                       delimiter=',',
                       converters={1: lambda x: codes.index(x.decode())})
    if tptxy.size == 0:
        raise RuntimeError(f'{log} is empty')
    txy = tptxy[:, 2:]
    penup = tptxy[:, 1]
    penup = (penup == codes.index('U'))

    # Cut up the stroke
    strokes = []
    cur = 0
    for ind in np.argwhere(penup).flatten() + 1:
        strokes.append(txy[cur:ind])
        cur = ind
    return strokes


def plot_txy(strokes):
    # Plot the strokes
    fig, ax = plt.subplots(figsize=(5, 5))
    for stroke in strokes:
        ax.plot(stroke[:, 1], stroke[:, 2])
    ax.grid(False)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_aspect('equal')
    return fig


def export(txys, name, outfolder=DEFAULT_OUTFOLDER):
    with open(outfolder / f'{name}.json', 'w') as f:
        f.write(load_gml.txy_to_gml_json(txys, [0, 1, 0, 1]))


def convert(logfile, outfolder=DEFAULT_OUTFOLDER, create_svg=False):
    try:
        strokes = to_txy(logfile)
    except RuntimeError as e:
        print(f'\033[93m\tWARNING: failed to convert {logfile} due to ', e,
              '\033[0m')
        return
    export(strokes, logfile.stem, outfolder=outfolder)
    if create_svg:
        fig = plot_txy(strokes)
        fig.savefig(outfolder / f'{logfile.stem}.svg')


def convert_all_remaining(infolder=DEFAULT_INFOLDER,
                          outfolder=DEFAULT_OUTFOLDER,
                          create_svg=False):
    for logfile in tqdm.tqdm(sorted(infolder.glob('*.txt'))):
        if (outfolder / f'{logfile.stem}.json').exists():
            print(f'{logfile.stem} already exists')
            continue
        with Stopwatch(name=f'Converting {logfile.stem}', print=False):
            convert(logfile, outfolder=outfolder, create_svg=create_svg)


def convert_all(infolder=DEFAULT_INFOLDER,
                outfolder=DEFAULT_OUTFOLDER,
                create_svg=False):
    for logfile in tqdm.tqdm(sorted(infolder.glob('*.txt'))):
        with Stopwatch(name=f'Converting {logfile.stem}', print=False):
            convert(logfile, outfolder=outfolder, create_svg=create_svg)


def generate_html(outfolder):
    TEMPLATE = '''
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>SVG Gallery</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 20px;
        }
        .svg-container {
            display: flex;
            flex-wrap: wrap;
            gap: 10px;
        }
        .svg-item {
            border: 1px solid #ccc;
            /* padding: 10px; */
            box-shadow: 0 0 10px rgba(0,0,0,0.1);
            text-align: center;
            flex-basis: calc(20% - 10px);
        }
        .svg-item img {
            max-width: 100%;
            height: auto;
        }
    </style>
</head>
<body>
    <h1>SVG Gallery</h1>
    <div class="svg-container">
        <!-- SVG items will be inserted here by Python script -->
        <!-- Example:
        <div class="svg-item">
            <img src="path/to/svg/file1.svg" alt="SVG 1">
        </div>
        -->
        #####SVGs#####
    </div>
</body>
</html>
'''
    div = lambda file: f'<div class="svg-item">{file.stem}<br /><img src="{file.name}" alt="{file.stem}"></div>'
    s = '\n'.join([div(file) for file in sorted(outfolder.glob('*.svg'))])
    with open(outfolder / '_gallery.html', 'w') as f:
        f.write(TEMPLATE.replace('#####SVGs#####', s))


def main():
    parser = argparse.ArgumentParser(description='Convert logs to GML')
    parser.add_argument('--infolder',
                        type=Path,
                        default=DEFAULT_INFOLDER,
                        help=f'Default: {DEFAULT_INFOLDER}')
    parser.add_argument('--outfolder',
                        type=Path,
                        default=DEFAULT_OUTFOLDER,
                        help=f'Default: {DEFAULT_OUTFOLDER}')
    parser.add_argument('-i', '--create_svg', action='store_true')
    parser.add_argument(
        '-a',
        '--all',
        action='store_true',
        help='Convert all logs, overwriting existing converted logs')
    parser.add_argument(
        '-b',
        '--all_unconverted',
        action='store_true',
        help='Convert all logs that have not been converted yet')
    parser.add_argument(
        '-l',
        '--log',
        type=Path,
        help='Convert a single log file (specify the file path)')
    parser.add_argument('-g',
                        '--generate_html',
                        action='store_true',
                        help='Generate an HTML gallery of the SVGs')
    args = parser.parse_args()

    args.outfolder.mkdir(exist_ok=True)

    if args.all:
        convert_all(infolder=args.infolder,
                    outfolder=args.outfolder,
                    create_svg=args.create_svg)
    elif args.all_unconverted:
        convert_all_remaining(infolder=args.infolder,
                              outfolder=args.outfolder,
                              create_svg=args.create_svg)
    elif args.log:
        convert(args.log, outfolder=args.outfolder, create_svg=args.create_svg)
    elif args.generate_html:
        generate_html(args.outfolder)
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
