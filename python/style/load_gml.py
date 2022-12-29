'''Load GML files.
@author Gerry
Example usage:
```
import load_gml
from load_gml import Drawing

files = Path('data/gml').glob('*.json')
files = load_gml.filter_by_application(files)  # filter out files not from Fat Tag
for file in files:
    drawing = Drawing(file)
    for stroke in drawing.strokes:
        print(stroke)
```

See also:
* GML Spec - https://fffff.at/gml/
* javascript canvas player example - https://jamiedubs.com/canvasplayer/?random
    * source - https://github.com/jamiew/canvasplayer
'''

import dataclasses
import json
import traceback
from pathlib import Path

import gerry00_gml_downloader
import numpy as np


@dataclasses.dataclass
class Drawing:
    '''Represents a GML drawing.'''

    def __init__(self, fname, **read_json_kwargs):
        '''Reads a GML JSON file and stores it as a Drawing object.'''
        # TODO(gerry): check for multiple strokes etc?
        # TODO(gerry): add support for other optional GML fields (e.g. color, brush, etc.)
        self.raw_dict = read_json(fname, **read_json_kwargs)
        self.id = self.raw_dict['id']
        self.strokes = self.raw_dict['gml']['tag']['drawing']


def get_from_blackbook(id, save_path=Path('data/gml'), **drawing_kwargs):
    '''Downloads a GML file from 000000book.com and returns it as a Drawing object.
    Args:
        id (int): The id of the file to download.
        save_path (pathlib.Path): The path to save the file to [Default: data/gml].
    '''
    fname = save_path / f'{id}.json'
    if not fname.exists():
        print(f'File {fname} does not exist, downloading...')
        gerry00_gml_downloader.download_one_blocking(id, save_path=save_path)
    return Drawing(fname, **drawing_kwargs)


def filter_by_application(iterable, application_name='Fat Tag - Katsu Edition'):
    '''Returns an iterable of files that are from the given application.'''
    return filter(lambda fname: is_from_application(fname, application_name), iterable)


def is_from_application(fname, application_name='Fat Tag - Katsu Edition'):
    '''Returns True if the given file is from the given application.'''
    with open(fname, 'r') as f:
        return json.load(f).get('gml_application') == application_name


def read_json(fname, verbosity=1):
    '''Reads a GML JSON file, replacing strokes from dicts to numpy arrays.  Returns as dict.'''
    with open(fname, 'r') as file:
        try:
            return json.load(file, object_hook=json_decoder_hook)
        except (KeyError, TypeError, AttributeError, AssertionError) as e:
            if verbosity >= 1:
                print(f'Error while reading {fname}: {type(e)}')
            if verbosity >= 2:
                traceback.print_exc()
            if verbosity >= 3:
                print(json.load(file))
                print('-' * 80)
            return None


def _float(x):
    if x is None:
        return float('nan')
    return float(x.replace(',', '.'))


def _pt2tuple(pt):
    '''Converts a GML point (as dict) to a tuple of (time, x, y, z).'''
    return _float(pt.get('time')), _float(pt['x']), _float(pt['y']), _float(pt.get('z'))


def json_decoder_hook(data):
    '''Custom json decoder hook to do some preprocessing to make the GML object easier to use.'''
    if 'pt' in data:
        # Convert a single stroke to a numpy array
        if isinstance(data['pt'], list):
            return np.array([_pt2tuple(pt) for pt in data['pt']])
        else:
            return np.array([_pt2tuple(data['pt'])])
    elif 'stroke' in data:
        # Convert list of strokes to array if it isn't already (due to GML->json conversion bug)
        return [data['stroke']] if isinstance(data['stroke'], np.ndarray) else data['stroke']
    if 'tag' in data:
        # scale canvas to screen size
        assert isinstance(data['tag'], dict), 'tag is not a dict.  GML malformed?'
        if 'environment' in data['tag']:
            w = float(data['tag']['environment']['screenBounds']['x'])
            h = float(data['tag']['environment']['screenBounds']['y'])
        else:
            w, h = 1, 1
        for stroke in data['tag']['drawing']:
            stroke[:, 1] *= w
            stroke[:, 2] *= h
        # swap x and y if the device is held in portrait mode
        if 'environment' in data['tag'] and 'up' in data['tag']['environment']:
            orientation = data['tag']['environment']['up']
            if float(orientation['x']):
                for stroke in data['tag']['drawing']:
                    stroke[:, [1, 2]] = stroke[:, [2, 1]]
            elif float(orientation['y']):
                for stroke in data['tag']['drawing']:
                    stroke[:, 2] = w - stroke[:, 2]
    return data
