import asyncio
import datetime
import websockets
import time


def get_local_ip():
    import socket
    return socket.gethostbyname(socket.gethostname())


HOST = get_local_ip()


async def echo(websocket):
    print('Connection opened!')
    s = time.perf_counter()
    t = s
    try:
        while True:
            msg = await websocket.recv()
            prev_t, t = t, time.perf_counter()
            print(f'{t - s:.2f}s', f'{1/(t-prev_t):.0f}Hz', msg)
            await websocket.send(msg)  # echo-back
    except websockets.exceptions.ConnectionClosedOK:
        print("Connection closed!")


async def main():
    async with websockets.serve(echo, HOST, 5679):
        await asyncio.Future()  # run forever


if __name__ == "__main__":
    asyncio.run(main())
