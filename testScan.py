import time
import asyncio
import bitstruct
import struct
import csv
import keyboard

from bleak import BleakScanner, BleakClient


HR_MEAS="00002A37-0000-1000-8000-00805F9B34FB"

def write_to_csv(data):
    print("Writed to csv")
    with open('heart_rate.csv', mode='a', newline='') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(data)

def hr_val_handler(sender,data):
    #print(f"Sender: {sender}")
    #print(f"Data: {data}")

    #print("HR Measurement Raw= {0}: {1}".format(sender,data))
    (hr_fmt, snsr_detect, snsr_cntct_spprtd, nrg_expnd, rr_int)= bitstruct.unpack("b1b1b1b1b1<", data)
    #print(f"hr_fmt: {hr_fmt}")
    #print(type(data))

    if hr_fmt:
        #print("Is hr_fmt")
        hr_val, = struct.unpack_from("<H", data, 1)
    else:
        hr_val, = struct.unpack_from("<B", data, 1)

    timestamp = int(time.time())
    write_to_csv([timestamp, hr_val])
    print(f"HR value {hr_val}")

async def run():
    devices = await BleakScanner.discover()
    if not devices:
        print("No devices found.")
        return

    print(devices)

    for d in devices:
        name = d.name.split()[0]
        if name in ["Polar"]:
            PolarMACAddress = d.address
            print(f"Discovering {name}: {d.address}")

    async with BleakClient(PolarMACAddress) as client:
        connected = await client.is_connected()
        print(f"Connected: {connected}")
        await client.start_notify(HR_MEAS, hr_val_handler)
        while await client.is_connected():
            await asyncio.sleep(1)

    with open('heart_rate.csv', mode='w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(['Timestamp', 'HR Value'])

     # Add sleep timer so that no overload BLE
#    while True:
#        time.sleep(1)
#        if keyboard.is_pressed('q'):
#            break
#        else:
#        await asyncio.sleep(0.1)

if __name__ == "__main__":
    asyncio.run(run())

