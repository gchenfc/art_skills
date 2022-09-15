'''fit_node.py
This file acts as a "node" that connects to a websocket server.  It receives drawing data, performs
regression/fitting, and sends the results back to the websocket server.
'''

# NOTES:
# How do we tell we are in a new trajectory?
# Need to pick the txy in and txy estimated

import asyncio
from copyreg import pickle
from fit_incremental import FixedLagFitter
import struct
import numpy as np
import pickle
from datetime import datetime

data_to_fit = []


def consume_data():
    tmp = np.array(data_to_fit)
    data_to_fit.clear()
    return tmp


async def update_fit(fitter, writer):
    if not fitter.initialized:
        if len(data_to_fit) > 10:
            print('Initializing fitter!!!')
            txy = iter(fitter.initialize(consume_data()))
            print('Fitter initialized!')
            writer.write('M'.encode() + struct.pack('ff', *next(txy)[1:]))
            for _, *xy in txy:
                writer.write('L'.encode() + struct.pack('ff', *xy))
    else:
        current_stroke = fitter.stroke_n
        print('Updating fitter!!!')
        check_segment(fitter)
        print("STROKES:",fitter.stroke_n)
        if fitter.stroke_n != current_stroke:
            _, x, y = fitter.step(consume_data(), False)[-1]
        else:
            _, x, y = fitter.step(consume_data(), False)[-1]
        print('Fitter updated!')
        print(fitter.history[-1][1][-1])
        writer.write('L'.encode() + struct.pack('ff', x, y))
    if len(fitter.history) > 0:
        return fitter.history[-1][1][-1]
    else:
        return None


def write_history(fitter):
    filename = datetime.now().strftime("%y_%m_%d-%H_%M_%S") + "_traj.p"
    filename = 'tmp.p'
    with open('trajectories/' + filename, 'wb') as f:
        pickle.dump(fitter.snr_history, f)
    print("\n\n _______________PICKLE_______________\n\n")


def check_segment(fitter):
    v = np.diff(fitter.full_stroke[:, 1:], axis=0) / np.diff(fitter.full_stroke[:,0]).reshape(-1,1)
    speed = np.sum(np.square(v), axis=1)
    acceleration = np.diff(speed, axis=0) / np.diff(fitter.full_stroke[1:,0])
    #print("\n\n\n___________________accel\n",acceleration)
    pre_infls = np.where(np.diff(np.sign(acceleration))>0)[0]
    if np.any(pre_infls):
        # print("___________________infls\n",pre_infls,pre_infls+1)
        fitter.stroke_n = np.size(pre_infls)

async def client():
    while True:
        try:
            reader, writer = await asyncio.open_connection('127.0.0.1', 5902)
            break
        except:
            print('Could not connect to fit server... trying again in 1 second')
            await asyncio.sleep(1)
    print('Connected to fit server!')

    fitter = FixedLagFitter()

    # TODO?: add trajectory end detection with force < 0.001 (or similar)

    fit_task = asyncio.create_task(asyncio.sleep(0.01))
    prev_t = -1
    while not reader.at_eof():
        data = await reader.read(13)
        c, (t, x, y) = data[:1].decode(), struct.unpack('fff', data[1:])
        # prevent dt = 0
        if (c != 'U') and (-0.0000001 < t - prev_t < 0.0015):
            continue
        prev_t = t
        if c == 'U':
            if fitter.initialized:
                last_txy = fitter.history[-1][1][-1]
                writer.write('U'.encode() + struct.pack('ff', *last_txy[1:]))
                await fit_task
                if len(data_to_fit) > 0:
                    await update_fit(fitter, writer)
                write_history(fitter)  # pickles data for SNR
        if c == 'M':
            fitter = FixedLagFitter()
            data_to_fit.clear()
        data_to_fit.append((t, x, y))
        if fit_task.done():
            fit_task = asyncio.create_task(update_fit(fitter, writer))
        print(f'{c} {x:.3f} {y:.3f}')

    print('Closing connection to fit server')
    writer.close()


def main():
    asyncio.run(client())


if __name__ == '__main__':
    main()
