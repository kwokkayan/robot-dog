import signal
import copy
from math import floor
import asyncio
from threading import Thread
from functools import reduce
from bleak import BleakScanner, BleakClient
from xbox360controller import Xbox360Controller

# Bluetooth const
ROBOT_NAME = "BLE2836"
MODEL_NBR_UUID = "0000dfb1-0000-1000-8000-00805f9b34fb"
ADDRESS = "AC:67:B2:3D:36:2A"
PKG_HEADER = b'\x55\xAA\x11'
# Controller const
AXIS_THRESHOLD = 0.01
BUTTON_MAP = {
              "button_y": 1,
              "button_b": 2,
              "button_a": 3,
              "button_x": 4,
              "button_select": 5,
              "button_start": 6
             }
button_pressed = [False for _ in range(6)]
axis_value = [128, 128]
### Controller helper functions
def debug_controller():
    print("=" * 20)
    print(axis_value)
    print(button_pressed)
    print("=" * 20)

def convert_axis(num):
    if (-AXIS_THRESHOLD < num < AXIS_THRESHOLD):
        return 128
    else:
        return floor(num * 127 + 128)

def on_button_pressed(button):
    button_pressed[BUTTON_MAP[button.name] - 1] = True

def on_button_released(button):
    button_pressed[BUTTON_MAP[button.name] - 1] = False

def on_axis_moved(axis):
    axis_value[0] = convert_axis(axis.x)
    axis_value[1] = convert_axis(-axis.y)

def bluetooth_main(address):
  # async in the thread job: WHY
  # https://youtu.be/RoSs9-NDP3E
  async def main(address):
      global controller_updated
      device = await BleakScanner.find_device_by_address(address)
      if (device == None):
        print("Cannot find {}".format(address))
        exit(1)
      async with BleakClient(device) as client:
          old_payload = b''          
          while True:
            # parsin'
            num_of_buttons_pressed = reduce(lambda a, b: a + b, button_pressed).to_bytes(1, "big")
            num_of_joystick_moved = b'\x00'
            button_data = b''
            for i in range(len(button_pressed)):
              if button_pressed[i]:
                button_data += (i+1).to_bytes(1, "big")
            joystick_data = axis_value[1].to_bytes(1, "big") + axis_value[0].to_bytes(1, "big") + b'\x00\x00' 
            payload = PKG_HEADER + num_of_buttons_pressed + num_of_joystick_moved + button_data + joystick_data
            checksum = reduce(lambda a, b: a + b, payload) % 256
            payload += checksum.to_bytes(1, "big")
            # sendin'
            if old_payload != payload:
              await client.write_gatt_char(MODEL_NBR_UUID, payload)
              old_payload = copy.deepcopy(payload)
            # sleepin'
            await asyncio.sleep(0.2)
  asyncio.run(main(address))

try:
  with Xbox360Controller(0, axis_threshold=AXIS_THRESHOLD) as controller:
    controller.button_a.when_pressed = on_button_pressed
    controller.button_a.when_released = on_button_released
    controller.button_b.when_pressed = on_button_pressed
    controller.button_b.when_released = on_button_released
    controller.button_x.when_pressed = on_button_pressed
    controller.button_x.when_released = on_button_released
    controller.button_y.when_pressed = on_button_pressed
    controller.button_y.when_released = on_button_released
    controller.button_start.when_pressed = on_button_pressed
    controller.button_start.when_released = on_button_released
    controller.button_select.when_pressed = on_button_pressed
    controller.button_select.when_released = on_button_released
    controller.axis_l.when_moved = on_axis_moved
    bluetooth_thread = Thread(target=bluetooth_main, args=(ADDRESS,))
    bluetooth_thread.start()
    signal.pause()
except KeyboardInterrupt:
  pass