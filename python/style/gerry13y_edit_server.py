import asyncio
import websockets
import pickle
import numpy as np
from gerry13y_edit import Editor

editor = None


# Define the function to be applied to the array
def func(array):
    # return array + [0.1, 0]
    return editor.edit([array], guidance_weight=1e3)[0]


async def server(websocket, path):
    print("Connected to client")
    while True:
        try:
            data = await websocket.recv()
            array = pickle.loads(data)
            print("Received array:", array)
            result = func(array)
            print("Processed array:", result)

            await websocket.send(pickle.dumps(result))
        except websockets.ConnectionClosed:
            print("Connection closed")
            break


def main():
    global editor
    editor = Editor()

    start_server = websockets.serve(server, "localhost", 5909)

    print(f"Server started at ws://localhost:5909")

    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()


if __name__ == "__main__":
    main()
