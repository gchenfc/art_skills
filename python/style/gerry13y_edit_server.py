import asyncio
import websockets
import pickle
import numpy as np
# from gerry13y_edit import Editor
from gerry10_edit import Editor

editor = None


# Define the function to be applied to the array
def func(array):
    # return [array + [0.1, 0]]
    # if array.shape[0] < 15:
    #     return [editor.edit([array], guidance_weight=1e5)[0]]
    # else:
    #     return [
    #         editor.edit([array], guidance_weight=1e2, repeat=3, t_start=10)[0]
    #     ]
    # return editor.edit([array], guidance_weight=1e6, repeat=3, t_start=10)
    return editor.edit([array], guidance_weight=1e1, repeat=20, t_start=3)


async def server(websocket, path):
    print("Connected to client")
    # editor.past_strokes = []
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
