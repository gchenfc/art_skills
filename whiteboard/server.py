'''server.py
Main server for real-time SLN demo.
* Receives drawing inputs, sends to SLN regression (TODO(gerry))
* Receives SLN regression results, sends to cable robot controller (TODO(gerry))

run with:
    watchmedo auto-restart --pattern "*.py" --recursive --signal SIGTERM python server.py

@author Gerry Chen
'''
import asyncio
import socket
import websockets
import time

PORTS = {
    'whiteboard_input': 5900,
    'whiteboard_fit': 5901,
    'fit_input': 5902,
    'fit_output': 5903,
    'robot_input': 5904,
    'robot_output': 5905
}


def get_local_ip():
    # return socket.gethostbyname(socket.gethostname())
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    return s.getsockname()[0]


HOST = get_local_ip()

clients = {'whiteboard': set(), 'fit': set(), 'robot': set()}


async def handle_whiteboard(websocket):
    print('Whiteboard connection opened!')
    s = time.perf_counter()
    t = s
    try:
        clients['whiteboard'].add(websocket)
        while True:
            msg = await websocket.recv()
            prev_t, t = t, time.perf_counter()
            print(f'{t - s:.2f}s', f'{1/(t-prev_t):.0f}Hz', msg)
            for writer in clients['fit']:
                writer.write((msg + '\n').encode())
                # await writer.drain()
            await websocket.send(msg)  # echo-back
    except websockets.exceptions.ConnectionClosedOK:
        print('Whiteboard connection closed!')
    finally:
        clients['whiteboard'].remove(websocket)


async def whiteboard_server():
    print(f'Serving whiteboard at: {HOST}:{PORTS["whiteboard_input"]}')
    async with websockets.serve(handle_whiteboard, HOST, PORTS['whiteboard_input']):
        await asyncio.Future()
    print('Closed whiteboard server')


async def handle_fit(reader, writer):
    print('Fit connection opened!')
    try:
        clients['fit'].add(writer)
        while not reader.at_eof():
            data = await reader.readline()
            message = data.decode()
            addr = writer.get_extra_info('peername')
            print(f"Received {message!r} from {addr!r}")
        print('Fit connection closed due to EOF!')

    except websockets.exceptions.ConnectionClosed:
        print('Fit connection closed!')
    finally:
        clients['fit'].remove(writer)


async def fit_server():
    server = await asyncio.start_server(handle_fit, 'localhost', PORTS['fit_input'])

    addrs = ', '.join(str(sock.getsockname()) for sock in server.sockets)
    print(f'Serving fit server at {addrs}')

    async with server:
        await server.serve_forever()

    print('Closed fit server')


async def main():
    await asyncio.wait([
        asyncio.create_task(whiteboard_server()),
        asyncio.create_task(fit_server()),
    ])


if __name__ == "__main__":
    print("STARTING UP!!!")
    asyncio.get_event_loop().run_until_complete(main())
