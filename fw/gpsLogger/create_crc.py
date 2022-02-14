#!/usr/bin/python

from crc import  CrcCalculator, Crc16
import struct
import sys, os

c = CrcCalculator(Crc16.CCITT)
with open(sys.argv[1], 'rb') as f:
    # Drop final bytes
    data = f.read()[:-2]

    v = c.calculate_checksum(bytes(data))
    print("crc: 0x{:04X}".format(v))

with open('{}.crc'.format(sys.argv[1]), 'wb') as f:
    f.write(struct.pack(">H", v))

    