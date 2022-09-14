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


def update_fit(fitter, writer):
    if not fitter.initialized:
        if len(data_to_fit) > 10:
            print('Initializing fitter!!!')
            txy = iter(fitter.initialize(consume_data()))
            print('Fitter initialized!')
            writer.write('M'.encode() + struct.pack('ff', *next(txy)[1:]))
            for _, *xy in txy:
                writer.write('L'.encode() + struct.pack('ff', *xy))
    else:
        print('Updating fitter!!!')
        _, x, y = fitter.step(consume_data(), False)[-1]
        print('Fitter updated!')
        # boo = np.array(fitter.history)
        print(fitter.history[-1][1][-1])
        writer.write('L'.encode() + struct.pack('ff', x, y))
    if len(fitter.history) > 0:
        return fitter.history[-1][1][-1]
    else:
        return None


def write_history(fitter):
    filename = datetime.now().strftime("%y_%m_%d-%H_%M_%S") + "_traj.p"
    with open(filename, 'wb') as f:
        pickle.dump(fitter.snr_history, f)
    f.close()
    print("\n\n _______________PICKLE_______________\n\n")


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

    while not reader.at_eof():
        data = await reader.read(13)
        c, (t, x, y) = data[:1].decode(), struct.unpack('fff', data[1:])
        if c == 'M':
            if fitter.initialized:
                last_txy = fitter.history[-1][1][-1]
                writer.write('U'.encode() + struct.pack('ff', *last_txy[1:]))                
                write_history(fitter) # pickles data for SNR
            fitter = FixedLagFitter()
            data_to_fit.clear()
        data_to_fit.append((t, x, y))
        res = update_fit(fitter, writer)
        print(f'{c} {x:.3f} {y:.3f} {res}')

    print('Closing connection to fit server')
    writer.close()


def main():
    asyncio.run(client())


if __name__ == '__main__':
    main()
