import loader
import websockets
import asyncio
import time
import numpy as np

async def hello():
    t0 = time.perf_counter()

    uri = "ws://192.168.0.15:5900"  # change this IP address to the one printed by server.py
    async with websockets.connect(uri) as websocket:
        data = loader.load_segments('D', index=None, debug=True)
        data = data[0] # comment-out this line if using loader.load_letter
        t = 0
        dt = 1./120
        async def wait_and_send(t, xy, stroketype='L'):
            t, x, y = xy # comment-out this line if using loader.load_letter
            if t > (time.perf_counter() - t0):
                await asyncio.sleep(t - (time.perf_counter() - t0))
            await websocket.send(f'{stroketype}{t},{x},{y}')

        for stroke in [np.vstack(data)]:
            await wait_and_send(t := t + dt, stroke[0], 'M')
            for xy in stroke[1:-1]:
                await wait_and_send(t := t + dt, xy)
            await wait_and_send(t := t + dt, stroke[-1], 'U')

if __name__ == '__main__':
    asyncio.run(hello())
