import serial
from serial.tools import list_ports
import struct
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-r", "--resolution", type=int)
args = parser.parse_args()

path = [p[0] for p in list_ports.comports() if "Mouse" in p[1]][0]
s = serial.Serial(path, 115200)

def set_reg(reg, val):
    data = struct.pack('BBB', reg, val, reg ^ val)
    s.write(data)
    s.flush()

if hasattr(args, 'resolution'):
    set_reg(0x0f, args.resolution)

s.close()
