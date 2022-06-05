# This code enables you to integrate a Bosch BMP180 temperature and pressure
# sensor by making use of the new Adafruit CircuitPython IO-drivers.
# It is derived in substantial parts from the predeccesor sensor (BMP085) code.
# The advantage of this new approach over the old code is, that it can be
# integrated in new projects that do not support the old IO-drivers any more.
# For example, the tca9548a i2c multiplexer works only with the new IO-drivers.
#
# Author: Johannes Beulshausen (ingenerd on github)
#
#
# Copyright (c) 2014 Adafruit Industries
# Author: Tony DiCola
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import time
import board
import busio
from adafruit_bus_device import i2c_device

# BMP180 default address (that can not be changed)
BMP180_I2CADDR           = 0x77


# Operating Modes for pressure speed and accuracy
BMP180_ULTRALOWPOWER     = 0
BMP180_STANDARD          = 1
BMP180_HIGHRES           = 2
BMP180_ULTRAHIGHRES      = 3


# BMP180 registers for calibration data (all 16 bits)
# AC1           = 0xAA
# AC2           = 0xAC
# AC3           = 0xAE
# AC4           = 0xB0
# AC5           = 0xB2
# AC6           = 0xB4
# B1            = 0xB6
# B2            = 0xB8
# MB            = 0xBA
# MC            = 0xBC
# MD            = 0xBE
# Therefore all consecutively stored values can be read out into an array of
# 11*2 bytes starting at register 0xAA!
BMP180_CAL      = 0xAA


# BMP180 registers to switch between temperature and pressure
# write "control" byte followed by either "readtempcmd" or "readpressurecmd"
# to read values
# first write "result register"
#     (tempdata, pressuredata (3 consecutive bytes for ULTRAHIGHRES))
# then read
BMP180_CONTROL           = 0xF4
BMP180_READTEMPCMD       = 0x2E
BMP180_READPRESSURECMD   = 0x34
BMP180_TEMPDATA          = 0xF6
BMP180_PRESSUREDATA      = 0xF6


#for interpretation of calibration from sensor
def notUnsigned(a):
    if a > 32767:
        a -= 65536
    return a


class BMP180:
    def __init__(
            self, i2c, address=BMP180_I2CADDR, mode=BMP180_STANDARD, **kwargs
            ):
        # Check that mode is valid.
        if mode not in [BMP180_ULTRALOWPOWER, BMP180_STANDARD, BMP180_HIGHRES, BMP180_ULTRAHIGHRES]:
            raise ValueError('Unexpected mode value {0}.  Set mode to one of BMP085_ULTRALOWPOWER, BMP085_STANDARD, BMP085_HIGHRES, or BMP085_ULTRAHIGHRES'.format(mode))

        self._address = address
        self._mode = mode
        self._i2c = i2c_device.I2CDevice(i2c, address)

        # Load calibration values.
        self._load_calibration()


    def _load_calibration(self):
        result = bytearray(22)
        self._i2c.write_then_readinto(bytes([BMP180_CAL]), result)

        self.cal_AC1 = (result[ 0]<<8) | result[ 1]   # INT16
        self.cal_AC2 = (result[ 2]<<8) | result[ 3]   # INT16
        self.cal_AC3 = (result[ 4]<<8) | result[ 5]   # INT16
        self.cal_AC4 = (result[ 6]<<8) | result[ 7]   # UINT16
        self.cal_AC5 = (result[ 8]<<8) | result[ 9]   # UINT16
        self.cal_AC6 = (result[10]<<8) | result[11]   # UINT16
        self.cal_B1  = (result[12]<<8) | result[13]   # INT16
        self.cal_B2  = (result[14]<<8) | result[15]   # INT16
        self.cal_MB  = (result[16]<<8) | result[17]   # INT16
        self.cal_MC  = (result[18]<<8) | result[19]   # INT16
        self.cal_MD  = (result[20]<<8) | result[21]   # INT16

        self.cal_AC1=notUnsigned(self.cal_AC1)
        self.cal_AC2=notUnsigned(self.cal_AC2)
        self.cal_AC3=notUnsigned(self.cal_AC3)

        self.cal_B1=notUnsigned(self.cal_B1)
        self.cal_B2=notUnsigned(self.cal_B2)
        self.cal_MB=notUnsigned(self.cal_MB)
        self.cal_MC=notUnsigned(self.cal_MC)
        self.cal_MD=notUnsigned(self.cal_MD)


    def read_raw_temp(self):
        """Reads the raw (uncompensated) temperature from the sensor."""
        result = bytearray(2)
        self._i2c.write(
            bytes([BMP180_CONTROL, BMP180_READTEMPCMD, BMP180_TEMPDATA])
            )
        time.sleep(0.005)
        self._i2c.readinto(result)
        raw=((result[0]<<8) | result[1])
        return raw


    def read_raw_pressure(self):
        """Reads the raw (uncompensated) pressure level from the sensor."""
        result = bytearray(3)
        self._i2c.write(
            bytes([BMP180_CONTROL, BMP180_READPRESSURECMD + (self._mode << 6),
            BMP180_PRESSUREDATA])
            )
        if self._mode == BMP180_ULTRALOWPOWER:
            time.sleep(0.005)
        elif self._mode == BMP180_HIGHRES:
            time.sleep(0.014)
        elif self._mode == BMP180_ULTRAHIGHRES:
            time.sleep(0.026)
        else:
            time.sleep(0.008)
        self._i2c.readinto(result)
        msb  = result[0]
        lsb  = result[1]
        xlsb = result[2]
        raw = ((msb << 16) + (lsb << 8) + xlsb) >> (8 - self._mode)
        return raw


    def Temp(self):
        """Gets the compensated temperature in degrees celsius."""
        UT = self.read_raw_temp()
        # Calculations below are taken straight from section 3.5 of the datasheet.
        X1 = ((UT - self.cal_AC6) * self.cal_AC5) >> 15
        X2 = (self.cal_MC << 11) // (X1 + self.cal_MD)
        B5 = X1 + X2
        temp = ((B5 + 8) >> 4) / 10.0
        return temp


    def Press(self):
        """Gets the compensated pressure in Pascals."""
        UT = self.read_raw_temp()
        UP = self.read_raw_pressure()
        # Calculations below are taken straight from section 3.5 of the datasheet.
        # Calculate true temperature coefficient B5.
        X1 = ((UT - self.cal_AC6) * self.cal_AC5) >> 15
        X2 = (self.cal_MC << 11) // (X1 + self.cal_MD)
        B5 = X1 + X2
        # Pressure Calculations
        B6 = B5 - 4000
        X1 = (self.cal_B2 * (B6 * B6) >> 12) >> 11
        X2 = (self.cal_AC2 * B6) >> 11
        X3 = X1 + X2
        B3 = (((self.cal_AC1 * 4 + X3) << self._mode) + 2) // 4
        X1 = (self.cal_AC3 * B6) >> 13
        X2 = (self.cal_B1 * ((B6 * B6) >> 12)) >> 16
        X3 = ((X1 + X2) + 2) >> 2
        B4 = (self.cal_AC4 * (X3 + 32768)) >> 15
        B7 = (UP - B3) * (50000 >> self._mode)
        if B7 < 0x80000000:
            p = (B7 * 2) // B4
        else:
            p = (B7 // B4) * 2
        X1 = (p >> 8) * (p >> 8)
        X1 = (X1 * 3038) >> 16
        X2 = (-7357 * p) >> 16
        p = p + ((X1 + X2 + 3791) >> 4)
        return p

    def Alt(self, sealevel_pa=101325.0):
        """Calculates the altitude in meters."""
        # Calculation taken straight from section 3.6 of the datasheet.
        pressure = float(self.Press())
        altitude = 44330.0 * (1.0 - pow(pressure / sealevel_pa, (1.0/5.255)))
        return altitude

    def PressNN(self, altitude_m=0.0):
        """Calculates the pressure at sealevel when given a known altitude in
        meters. Returns a value in Pascals."""
        pressure = float(self.Press())
        p0 = pressure / pow(1.0 - altitude_m/44330.0, 5.255)
        return p0

# ----
# Class BMP180
