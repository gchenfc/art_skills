# This is just a simple testing file.
# To test, run gerry13y_edit_server.py first, then run this file.
# To run live editing, run gerry13y_edit_server.py first, then run art_skills/whiteboard/server.py
import asyncio
import websockets
import pickle
import numpy as np


async def client():
    uri = "ws://localhost:5909"
    async with websockets.connect(uri) as websocket:
        # Send multiple arrays
        for i in range(3):
            array = np.random.randn(100, 3)
            print("Sending array:", array)

            await websocket.send(pickle.dumps(array))

            data = await websocket.recv()
            result = pickle.loads(data)
            print("Received processed array:", result)


asyncio.get_event_loop().run_until_complete(client())
