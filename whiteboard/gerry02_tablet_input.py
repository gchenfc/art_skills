import pyglet
import time
import websockets
import asyncio
import pickle

import numpy as np
import matplotlib.pyplot as plt
import time

t0 = time.perf_counter()

def get_tablet():
    tablet = None
    for device in pyglet.input.get_devices():
        if device.name is None or 'Tablet' not in device.name:
            print(device.name)
            continue
        print(device.name, type(device), device.get_controls())
        if len(device.get_controls()) > 5:
            tablet = device
            break
    return tablet


def get_controls(tablet):
    controls = tablet.get_controls()
    return controls


send_buffer = []

# socket = websockets.connect('ws://192.168.0.15:5900')
# async def whiteboard_client():
#     async with websockets.connect('ws://192.168.0.15:5900') as socket:
#         while True:
#             if len(send_buffer) > 0:
#                 await socket.send(send_buffer.shift())
#             # await asyncio.sleep(0.001)

def callback(controls, debug=False):
    """Returns (x, y, ispressed)"""
    # 0: Pen touching tablet (bool - ispressed)
    # 1: Pen button 1
    # 2: Pen button 2
    # 3-6: ???
    # 7: x
    # 8: y (top is 0)
    # 9: pen pressure
    c_x, c_y = controls[-3], controls[-2]
    x = (c_x.value - c_x.min) / (c_x.max - c_x.min)
    y = 1 - (c_y.value - c_y.min) / (c_y.max - c_y.min)
    if debug:
        print(x, y)
        for control in controls:
            if isinstance(control, pyglet.input.base.Button):
                print(control.value, end='\t')
            else:
                print(control.value, end='\t')  # control.min, control.max
        print()
    return x, y, controls[0].value or controls[1].value or controls[2].value

def main():
    # client_task = asyncio.create_task(whiteboard_client())

    tablet = get_tablet()
    controls = get_controls(tablet)

    # Set up gui
    # window = pyglet.window.Window(fullscreen=True)
    window = pyglet.window.Window(640, 480)

    try:
        canvas = tablet.open(window)
    except pyglet.input.DeviceException:
        print('Failed to open tablet on window')
        return
    else:
        print('Opened tablet on window')

    @window.event
    def on_key_press(symbol, modifiers):
        if symbol == ord('w') and (modifiers & pyglet.window.key.MOD_ACCEL):  # cmd-W
            with open('/tmp/data.pickle', 'wb') as f:
                pickle.dump(send_buffer, f)
            window.close()

        if symbol == ord('q'):
            with open('/tmp/data.pickle', 'wb') as f:
                pickle.dump(send_buffer, f)
            window.close()

    prev_pressed = False
    # socket = await websockets.connect('ws://192.168.0.15:5900')
    # print(socket)

    t_prev = t0 - 1000

    @controls[-2].event
    @controls[-3].event
    @controls[-1].event
    def on_change(_):
        nonlocal t_prev
        tnow = time.perf_counter()
        # if tnow - t_prev < 0.005:
        #     return
        t_prev = tnow
        nonlocal prev_pressed
        x, y, is_pressed = callback(controls)
        print(x, y)
        if is_pressed:
            # update_cablerobot(robot, x, y)
            print(x, y)
            if prev_pressed:
                send_buffer.append(f'L{tnow - t0},{x},{y}')
            else:
                send_buffer.append(f'M{tnow - t0},{x},{y}')
        else:
            if prev_pressed:
                send_buffer.append(f'U{tnow - t0},{x},{y}')
        prev_pressed = is_pressed
    # def update(dt):
    #     x, y, is_pressed = callback(controls)
    #     print(x, y)
    #     if is_pressed:
    #         update_cablerobot(robot, x, y)
    #         print(x, y)
    #     robot.update()
    # pyglet.clock.schedule_interval(update, 0.0003)

    pyglet.app.run()


if __name__ == '__main__':
    main()
