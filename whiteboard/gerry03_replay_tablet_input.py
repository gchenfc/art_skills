import pickle
import time
import websockets
import asyncio


async def hello():
    with open('/tmp/data.pickle', 'rb') as f:
        send_buffer = pickle.load(f)
    t0 = time.perf_counter()

    uri = "ws://192.168.0.15:5900"
    async with websockets.connect(uri) as websocket:
        for row in send_buffer:
            t = float(row[1:].split(',')[0])
            if t > (time.perf_counter() - t0):
                await asyncio.sleep(t - (time.perf_counter() - t0))
            await websocket.send(row)
            print(row)

if __name__ == '__main__':
    asyncio.run(hello())
