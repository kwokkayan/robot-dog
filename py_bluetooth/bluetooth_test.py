import asyncio
from functools import reduce
from bleak import BleakScanner, BleakClient
# Bluetooth
ROBOT_NAME = "ESP-32"
MODEL_NBR_UUID = "0000dfb1-0000-1000-8000-00805f9b34fb"
ADDRESS = "AC:67:B2:3D:36:2A"
# Packets
HEADER = b'\x55\xAA\x11'
#55 AA 11 00 03 80 80 00 00 13
async def scanner():
    devices = await BleakScanner.discover()
    for d in devices:
      print("{} {}".format(d.address, d.name))
      if (d.name != None):
         print(d)

async def main(address):
    device = await BleakScanner.find_device_by_address(address)
    if (device == None):
      print("Cannot find {}".format(address))
      exit(1)

    async with BleakClient(device) as client:
      c = client.services.characteristics
      for k in c:
         print(c[k])
      # num of button, array of button ids
      # data = HEADER + b'\x03\x02' + b'\x01\x01\x01' + b'\x00\xFF\x00\x00'
      # joy stick header is useless lol
      # data = HEADER + b'\x00\x00\xFF\xFF\x00\x00'
      # checksum = reduce(lambda a, b: a + b, data) % 256
      # data += checksum.to_bytes(1, "big")
      # print(data)
      # await client.write_gatt_char(MODEL_NBR_UUID, data)     

# asyncio.run(main(ADDRESS))
asyncio.run(scanner())