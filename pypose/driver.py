#!/usr/bin/env python3

"""
  PyPose: Serial driver for connection to arbotiX board or USBDynamixel.
  Copyright (c) 2008,2009 Michael E. Ferguson.  All right reserved.

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software Foundation,
  Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
"""

import serial
from pypose import ax12


class Driver:
    """ Class to open a serial port and control AX-12 servos
    through an arbotiX board or USBDynamixel. """

    def __init__(self, port="/dev/ttyUSB0", baud=1000000, interpolation=False, direct=False):
        """ This may throw errors up the line -- that's a good thing. """
        self.ser = serial.Serial()
        self.ser.baudrate = baud
        self.ser.port = port
        self.ser.timeout = 0.5
        self.ser.open()
        self.error = 0
        self.hasInterpolation = interpolation
        self.direct = direct

    def execute(self, index, ins, params):
        """ Send an instruction to a device. """
        self.ser.flushInput()
        packet = ""
        length = 2 + len(params)
        checksum = 255 - ((index + length + ins + sum(params)) % 256)
        packet += chr(0xFF) + chr(0xFF) + chr(index) + chr(length) + chr(ins)
        for val in params:
            packet += chr(val)
        packet += chr(checksum)
        self.ser.write(packet.encode())
        return self.getPacket(0)

    def setReg(self, index, regstart, values):
        """ Set the value of registers. Should be called as such:
        ax12.setReg(1,1,(0x01,0x05)) """
        self.execute(index, ax12.AX_WRITE_DATA, [regstart] + values)
        return self.error

    def getPacket(self, mode, id=-1, leng=-1, error=-1, params=None):
        """ Read a return packet, iterative attempt """
        # need a positive byte
        d = self.ser.read().decode('utf-8', 'backslashreplace')
        if d == '':
            print("Fail Read")
            return None

        # now process our byte
        if mode == 0:           # get our first 0xFF
            if ord(d) == 0xff:
                print("Oxff found")
                return self.getPacket(1)
            else:
                print("Oxff NOT found, restart: " + str(ord(d)))
                return self.getPacket(0)
        elif mode == 1:         # get our second 0xFF
            if ord(d) == 0xff:
                print("Oxff found")
                return self.getPacket(2)
            else:
                print("Oxff NOT found, restart: " + str(ord(d)))
                return self.getPacket(0)
        elif mode == 2:         # get id
            if d != 0xff:
                print("ID found: " + str(ord(d)))
                return self.getPacket(3, ord(d))
            else:
                print("0xff is not ID, restart")
                return self.getPacket(0)
        elif mode == 3:         # get length
            print("Length found: " + str(ord(d)))
            return self.getPacket(4, id, ord(d))
        elif mode == 4:         # read error
            print("Error level found: " + str(ord(d)))
            self.error = ord(d)
            if leng == 2:
                return self.getPacket(6, id, leng, ord(d), list())
            else:
                return self.getPacket(5, id, leng, ord(d), list())
        elif mode == 5:         # read params
            print("Parameter found: " + str(ord(d)))
            params.append(ord(d))
            if len(params) + 2 == leng:
                return self.getPacket(6, id, leng, error, params)
            else:
                return self.getPacket(5, id, leng, error, params)
        elif mode == 6:         # read checksum
            print("Checksum found: " + str(ord(d)))
            checksum = id + leng + error + sum(params) + ord(d)
            print("Checksum computed: " + str(checksum))
            if checksum % 256 != 255:
                print("Checksum ERROR")
                return None
            return params
        # fail
        return None

    def getReg(self, index, regstart, rlength):
        """ Get the value of registers, should be called as such:
        ax12.getReg(1,1,1) """
        vals = self.execute(index, ax12.AX_READ_DATA, [regstart, rlength])
        if vals is None:
            print("Read Failed: Servo ID = " + str(index))
            return -1
        if rlength == 1:
            return vals[0]
        return vals

    def syncWrite(self, regstart, vals):
        """ Set the value of registers. Should be called as such:
        ax12.syncWrite(reg, ((id1, val1, val2), (id2, val1, val2))) """
        self.ser.flushInput()
        length = 4
        valsum = 0
        packet = ""
        for i in vals:
            length = length + len(i)
            valsum = valsum + sum(i)
        checksum = 255 - ((254 + length + ax12.AX_SYNC_WRITE +
                           regstart + len(vals[0]) - 1 + valsum) % 256)
        # packet: FF FF ID LENGTH INS(0x03) PARAM .. CHECKSUM
        packet += chr(0xFF) + chr(0xFF) + chr(0xFE) + chr(length) + chr(ax12.AX_SYNC_WRITE) + chr(regstart) + chr(len(vals[0]) - 1)
        for servo in vals:
            for value in servo:
                packet += chr(value)
        packet += chr(checksum)
        self.ser.write(packet.encode())
        # no return info...
