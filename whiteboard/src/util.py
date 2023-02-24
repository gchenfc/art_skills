"""
util.py
Utility functions for handling stroke data.
@author Gerry
"""
import numpy as np
import matplotlib.pyplot as plt
import json


def derivative(t, x):
    """Compute the derivative of x w.r.t. t.  x/t are column-vectors (or mat)
    """
    xdot = np.hstack(tuple(np.gradient(x[:, i], t).reshape(-1, 1) for i in range(2)))
    return xdot


def derivatives(t, x):
    """Computes both the velocity and acceleration of x w.r.t. t.  x/t are column."""
    xdot = derivative(t, x)
    xddot = derivative(t, xdot)
    return xdot, xddot


def clean_stroke(stroke):
    """Clean up the data by removing duplicate points.  Returns a copy."""
    bad = np.diff(stroke[:, 0], axis=0) == 0
    bad = np.hstack(([False], bad))
    return stroke[~bad]


def clean_strokes(strokes):
    """Clean up the data by removing duplicate points.  Returns a copy."""
    return list(map(clean_stroke, strokes))


def stroke2tx(stroke):
    return stroke[:, 0], stroke[:, 1:]


def strokes2txs(strokes):
    return list(map(stroke2tx, strokes))


def plot_xva(axes, t, x, xdot, xddot, **line_kwargs):
    axes[0].plot(t, x, '-o', label=('x', 'y'), **line_kwargs)
    axes[1].plot(t, xdot, '-o', label=('x', 'y'), **line_kwargs)
    axes[2].plot(t, xddot, '-o', label=('x', 'y'), **line_kwargs)
    axes[0].set_ylabel('Position')
    axes[1].set_ylabel('Velocity')
    axes[2].set_ylabel('Acceleration')
    axes[0].legend()


def create_html_anim(*txs, fname=None, bnds=None, markersize=3):
    """Creates an html animation using the given strokes.
    Usage:
        create_html_anim((t1, x1), (t2, x2), ..., fname='test.html')
    Args:
        bnds: (min, max) where min and max are each 2-vectors representing the x/y min/max bounds.
        markersize (float): size of the markers in the animation.
    """
    if bnds is None or bnds == 'auto':
        all_xs = np.vstack([tx[1] for tx in txs])
        bnds = [np.min(all_xs, axis=0), np.max(all_xs, axis=0)]
        bnds = [bnds[0] - abs(0.1 * bnds[0]), bnds[1] + abs(0.1 * bnds[1])]  # padding
    if bnds == 'cablerobot':
        bnds = [np.array([0, 0]), np.array([2.9, 2.32])]  # cable robot
    if bnds == 'air':
        bnds = [np.array([0, 0]), np.array([6.17, 2.64])]  # cable robot
    axis_height, axis_width = 300, 300 * (bnds[1] - bnds[0])[0] / (bnds[1] - bnds[0])[1]
    string_to_write = ''
    def write(*lines):
        nonlocal string_to_write
        string_to_write += '\n'.join(lines) + '\n'
    # Create canvas
    write(
        f'<button onclick="run()">Run</button>'
        f'<canvas id="c" width="{axis_width * len(txs) + 1}" height="{axis_height}"></canvas>',
        f'<a id="link"></a>',
        f'<a href="?download=true" id="download">Download as png files (only works in-browser)</a>',
    )
    # Start javascript
    write(
        f"<script>",
        # f'    canvas = document.getElementById("c");',
        f'    canvases = document.getElementsByTagName("canvas");',
        f'    canvas = canvases[canvases.length - 1];',
        f'    ctx = canvas.getContext("2d");',
        f'    function run() {{',
        f'        ctx.clearRect(0, 0, canvas.width, canvas.height);',
        f'        ctx.fillStyle = "red";',
        f'',
    )
    write( # outlines for each txy sub-canvas
        f'        ctx.beginPath();',
        *[
            f'        ctx.rect({sx}, {0}, {axis_width}, {axis_height});'
            for sx in np.arange(0, axis_width * len(txs), axis_width)
        ],
        f'        ctx.stroke();',
        f'',
    )
    # Hardcode data into javascript and loop over data w/ timeouts
    max_time = 0
    for i, tx in enumerate(txs):
        t, x = tx[0], tx[1]  # don't unpack as `t, x = tx` in case txva is given.
        x = (x - bnds[0]) / (bnds[1] - bnds[0])
        max_time = max(max_time, t[-1])
        data = np.hstack((t.reshape(-1, 1) - t[0], x))
        write(
            f'        const data{i} = {json.dumps(data.tolist())};',
            f'        data{i}.forEach(element => {{',
            f'            setTimeout(() => {{',
            f'                console.log(element[1], element[2]);',
            f'                ctx.fillRect((element[1] + {i}) * {axis_width}, (1 - element[2]) * {axis_height}, {markersize}, {markersize});',
            f'            }}, element[0] * 1000);',
            f'        }});',
            f'',
        )
    write(
        '    }',
        '    run();',
        '',
    )
    # Export as 10Hz animation
    write(
        """    var params = new Proxy(new URLSearchParams(window.location.search), {""",
        """        get: (searchParams, prop) => searchParams.get(prop),""",
        """    });""",
        """    if (params.download) {""",
        """        var link = document.getElementById('link');""",
        """        var c = 0;""",
        """        var running = true;""",
        f"        setTimeout(() => {{ running = false; }}, {max_time * 1000});",
        """        var intervalId = setInterval(function () {""",
        """            if (running) {""",
        """                console.log('still running: ', running);""",
        """                // page.render('temp/screenshot' + String(a).padStart(3, '0') + '.png');""",
        """                link.setAttribute('download', 'screenshot' + String(c).padStart(3, '0') + '.png');""",
        """                link.setAttribute('href', canvas.toDataURL("image/png").replace("image/png", "image/octet-stream"));""",
        """                link.click();""",
        """            } else {""",
        """                clearInterval(intervalId);""",
        """                console.log('still running: ', running);""",
        """                phantom.exit(0);""",
        """            };""",
        """            c++;""",
        """        }, 100);""",
        """     }""",
        """</script>""",
    )

    if fname is not None:
        with open(fname, 'w') as f:
            f.write(string_to_write)

    return string_to_write
