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

from . import gerry00_gml_downloader
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
        self.fname = fname

    def __repr__(self):
        return f'Drawing(id={self.id}, num_strokes={len(self.strokes)})'


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
    print(fname, drawing_kwargs)
    return Drawing(fname, **drawing_kwargs)


def filter_by_application(iterable,
                          application_name='Fat Tag - Katsu Edition'):
    '''Returns an iterable of files that are from the given application.'''
    return filter(lambda fname: is_from_application(fname, application_name),
                  iterable)


def is_from_application(fname, application_name='Fat Tag - Katsu Edition'):
    '''Returns True if the given file is from the given application.'''
    with open(fname, 'r') as f:
        return json.load(f).get('gml_application') == application_name


def read_json(fname, verbosity=1, **hook_kwargs):
    '''Reads a GML JSON file, replacing strokes from dicts to numpy arrays.  Returns as dict.'''
    with open(fname, 'r') as file:
        try:
            if hook_kwargs is not None:
                return json.load(
                    file,
                    object_hook=lambda x: json_decoder_hook(x, **hook_kwargs))
            else:
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
    return _float(pt.get('time')), _float(pt['x']), _float(pt['y']), _float(
        pt.get('z'))


def json_decoder_hook(data, scale_behavior='GML_SCREEN'):
    '''Custom json decoder hook to do some preprocessing to make the GML object easier to use.
    scale_behavior: one of the following options:
        * 'GML_SCREEN' - scale to the GML screen size (true to the GML spec)
        * 'PRESERVE_ASPECT_CENTERED' - largest dimension is scaled to 1, and aspect ratio preserved
        * 'PRESERVE_ASPECT' - largest dimension is scaled to 1, and aspect ratio preserved (not centered)
        * 'NONE' - no scaling, so both x and y are in the range [0, 1]
    '''
    if 'pt' in data:
        # Convert a single stroke to a numpy array
        if isinstance(data['pt'], list):
            return np.array([_pt2tuple(pt) for pt in data['pt']])
        else:
            return np.array([_pt2tuple(data['pt'])])
    elif 'stroke' in data:
        # Convert list of strokes to array if it isn't already (due to GML->json conversion bug)
        return [data['stroke']] if isinstance(data['stroke'],
                                              np.ndarray) else data['stroke']
    if 'tag' in data:
        # scale canvas to screen size
        assert isinstance(data['tag'],
                          dict), 'tag is not a dict.  GML malformed?'
        if 'header' in data['tag'] and 'environment' in data['tag']['header']:
            data['tag']['environment'] = data['tag']['header']['environment']
        if 'environment' in data['tag']:
            w = float(data['tag']['environment']['screenBounds']['x'])
            h = float(data['tag']['environment']['screenBounds']['y'])
        else:
            w, h = 1, 1
        # Flatten the drawing list by 1 level
        if isinstance(data['tag']['drawing'][0], list):
            data['tag']['drawing'] = sum(data['tag']['drawing'], [])
        for stroke in data['tag']['drawing']:
            if scale_behavior == 'GML_SCREEN':
                stroke[:, 1] *= w
                stroke[:, 2] *= h
            elif scale_behavior == 'PRESERVE_ASPECT_CENTERED':
                scale = max(w, h)
                stroke[:, 1] = stroke[:, 1] * w / scale + (1 - w / scale) / 2
                stroke[:, 2] = stroke[:, 2] * h / scale + (1 - h / scale) / 2
            elif scale_behavior == 'PRESERVE_ASPECT':
                scale = max(w, h)
                stroke[:, 1] *= w / scale
                stroke[:, 2] *= h / scale
            elif scale_behavior == 'NONE':
                pass
            else:
                raise ValueError(f'Invalid scale_behavior: {scale_behavior}')
            data['w'], data['h'] = w, h
        # swap x and y if the device is held in portrait mode
        if 'environment' in data['tag'] and 'up' in data['tag']['environment']:
            orientation = data['tag']['environment']['up']
            if float(orientation['x']):
                for stroke in data['tag']['drawing']:
                    stroke[:, [1, 2]] = stroke[:, [2, 1]]
                    data['w'], data['h'] = h, w
            elif float(orientation['y']):
                if scale_behavior == 'GML_SCREEN':
                    for stroke in data['tag']['drawing']:
                        stroke[:, 2] = w - stroke[:, 2]
                elif scale_behavior == 'PRESERVE_ASPECT':
                    for stroke in data['tag']['drawing']:
                        stroke[:, 2] = (w / max(w, h)) - stroke[:, 2]
                else:
                    for stroke in data['tag']['drawing']:
                        stroke[:, 2] = 1 - stroke[:, 2]
    return data


def txy_to_gml_xml(txys, bounds):
    """Returns an xml string representing the given trajectory in GML format.
    Args:
        txys (Iterable[np.ndarray, nx3]): The trajectory to format (sequence of strokes)
        bounds (4-tuple, [xmin, xmax, ymin, ymax]): The bounds of the canvas
    """
    with open('data/gml_template.xml', 'r') as f:
        template = f.read()

    def to_xml(**kwargs):
        return '\n'.join([f'<{k}>{v}</{k}>' for k, v in kwargs.items()])

    UP_VECTOR3 = to_xml(x=0, y=1, z=0)
    SCREENBOUNDS = to_xml(x=bounds[1] - bounds[0], y=bounds[3] - bounds[2])
    for txy in txys:
        txy[:, 1:] += [bounds[0], bounds[2]]

    def to_stroke(stroke):
        return ('<stroke>' + '\n'.join(
            [to_xml(pt=to_xml(t=t, x=x, y=y, z=0))
             for t, x, y in stroke]) + '</stroke>')

    DRAWING = '\n'.join([to_stroke(stroke) for stroke in txys])

    return template.format(UP_VECTOR3=UP_VECTOR3,
                           SCREENBOUNDS_VECTOR2=SCREENBOUNDS,
                           DRAWING=DRAWING)


def txy_to_gml_json(txys, bounds, alt_format=False):
    """Returns a json string representing the given trajectory in GML format.
    Args:
        txys (Iterable[np.ndarray, nx3]): The trajectory to format (sequence of strokes)
        bounds (4-tuple, [xmin, xmax, ymin, ymax]): The bounds of the canvas
    """

    w, h = bounds[1] - bounds[0], bounds[3] - bounds[2]

    if not alt_format:
        drawing = [{
            "stroke": {
                "pt": [{
                    "time": str(t),
                    "x": str(x / w),
                    "y": str(y / h)
                } for t, x, y in stroke]
            }
        } for stroke in txys]
    else:
        drawing = {
            "stroke": [{
                "pt": [{
                    "time": str(t),
                    "x": str(x / w),
                    "y": str(y / h)
                } for t, x, y in stroke]
            } for stroke in txys]
        }

    ret = {
        "id": -1,
        "gml": {
            "tag": {
                "header": {
                    "filename": "temptTag-2009_8_23_13_21_12.gml",
                    "client": {
                        "name": "eyeSaver-003"
                    },
                    "environment": {
                        "screenBounds": {
                            "x": str(bounds[1] - bounds[0]),
                            "y": str(bounds[3] - bounds[2]),
                        },
                        "origin": {
                            "x": "0",
                            "y": "0",
                        },
                        # "up": {
                        #     "x": "0",
                        #     "y": "1",
                        #     "z": "0",
                        # },
                    }
                },
                "drawing": drawing
            }
        }
    }
    return json.dumps(ret)


def compute_state(stroke):
    return stroke[:, 1:3].astype(np.float32)


def compute_action(stroke):
    return np.diff(stroke[:, 1:3], axis=0,
                   append=stroke[-1:, 1:3]).astype(np.float32)


def compute_obs(stroke):
    return None
