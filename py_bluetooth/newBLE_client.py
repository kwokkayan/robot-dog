import signal
import copy
from math import floor
import asyncio
from threading import Thread
from functools import reduce
from bleak import BleakScanner, BleakClient
from xbox360controller import Xbox360Controller
import click

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
# Click const
UP_ASCII = b'\x1b[A'
DOWN_ASCII = b'\x1b[B'
ENTER_ASCII = b'\r'
ESCAPE_ASCII = b'\x1b'
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

### Click
def getKeyInAscii():
  return bytes(click.getchar(False), "ascii")

### Bluetooth
async def bluetooth_scanner():
  devices = await BleakScanner.discover()
  devices = list(filter(lambda d: d.name.replace("-", ":") != d.address, devices))
  choice = 0
  click.clear()
  while True:
    for i in range(len(devices)):
      d = devices[i]
      if (choice == i):
        click.echo(click.style("{} {}".format(d.address, d.name), fg="green"))
      else:
        click.echo("{} {}".format(d.address, d.name))
    click.echo("UP and DOWN to move cursor. ENTER to select. ESCAPE to exit.")
    # LL(0) Parser
    key = getKeyInAscii()
    if key == UP_ASCII:
      choice = (choice - 1) % len(devices)
    elif key == DOWN_ASCII:
      choice = (choice + 1) % len(devices)
    elif key == ENTER_ASCII:
      break
    elif key == ESCAPE_ASCII:
      raise Exception("Exited Program.")
    click.clear()
  return devices[choice]

def bluetooth_main(device):
  # async in the thread job: WHY
  # https://youtu.be/RoSs9-NDP3E
  async def main(device):
      global controller_updated
      async with BleakClient(device) as client:
          c = client.services.characteristics
          # select characteristic for GoBLE
          click.clear()
          keys = list(c.keys())
          choice = 0
          while True:
            for k in range(len(keys)):
              if choice == k:
                 click.echo(click.style(c[keys[k]], fg="green"))
              else:
                 click.echo(c[keys[k]])
            # LL(0) Parser
            key = getKeyInAscii()
            if key == UP_ASCII:
              choice = (choice - 1) % len(keys)
            elif key == DOWN_ASCII:
              choice = (choice + 1) % len(keys)
            elif key == ENTER_ASCII:
              break
            elif key == ESCAPE_ASCII:
              raise Exception("Exited Program.")
            click.clear()
          click.clear()
          GATT_char_UUID = c[keys[choice]]
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
              await client.write_gatt_char(GATT_char_UUID, payload)
              old_payload = copy.deepcopy(payload)
            # sleepin'
            await asyncio.sleep(0.2)
  asyncio.run(main(device))

def main():
  try:
    GoBLE_device = asyncio.run(bluetooth_scanner())
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
      bluetooth_thread = Thread(target=bluetooth_main, args=(GoBLE_device,))
      bluetooth_thread.start()
      signal.pause()
  except KeyboardInterrupt:
    pass
  except Exception as err:
    print(err)

if __name__ == "__main__":
   main()