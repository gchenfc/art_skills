'''fit_node.py
This file acts as a "node" that connects to a websocket server.  It receives drawing data, performs
regression/fitting, and sends the results back to the websocket server.
'''

import asyncio


async def client():
    while True:
        try:
            reader, writer = await asyncio.open_connection('127.0.0.1', 5902)
            break
        except:
            print('Could not connect to fit server... trying again in 1 second')
            await asyncio.sleep(1)
    print('Connected to fit server!')

    while not reader.at_eof():
        data = await reader.read(1)
        print(data.decode(), end='', flush=True)

    print('Closing connection to fit server')
    writer.close()


def main():
    asyncio.run(client())


if __name__ == '__main__':
    main()
