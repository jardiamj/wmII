#!/usr/bin/env python
#
# Copyright 2018 Jardi A. Martinez Jordan <jardiamj@gmail.com>
# See the file LICENSE.txt for your rights.
# Derived from Ultimeter driver.
# Most methods were derived from The Weather Update app by:
#   Scott Hassan <hassan@dotfunk.com>
#
#//////////////////////////////////////////////////////////////////////////////
# Adapted for Python3 Jay R. Jaeger  cube1us@gmail.com June, 2021
# with additional tests added for getting and setting station time
#
# REQUIRES: Python3 (tested under 3.8), weewx Version 4 (or later)
#
# Note:  I left ord(single-byte-value), though strictly speaking
# it should probably be done as int(single-byte-value)
#

"""Driver for Davis Weather Monitor II weather station, Should also work with
Wizzard and Perception but hasn't been tested


"""

import serial
import syslog
import time
import datetime
import struct
import logging

import weewx.drivers
import weewx.wxformulas
import weewx.crc16

from weewx.units import INHG_PER_MBAR, MILE_PER_KM
from weeutil.weeutil import timestamp_to_string

DRIVER_NAME = "wmII"
DRIVER_VERSION = "1.0"

log = logging.getLogger(__name__)

def loader(config_dict, _):
    return WMII(**config_dict[DRIVER_NAME])


def confeditor_loader():
    return WMIIConfEditor()


# def logmsg(level, msg):
#    log.info(level, "Weather Monitor II: %s" % msg)


def logdbg(msg):
    log.debug("Weather Monitor II: " + msg)


def loginf(msg):
    log.info("Weather Monitor II: " + msg)


def logerr(msg):
    log.error("Weather Monitor II: " + msg)


class WMII(weewx.drivers.AbstractDevice):
    """weewx driver for the Davis Wather Monitor II station

    model: station model, e.g., 'Weather Monitor II' or 'Wizard III'
    [Optional. Default is 'Weather Monitor II']

    port - serial port
    [Required. Default is /dev/ttyUSB0]

    loop_interval - The time (in seconds) between LOOP packets.
    [Optional. Default is 1]

    max_tries - how often to retry serial communication before giving up
    [Optional. Default is 5]
    """

    def __init__(self, **stn_dict):
        self.model = stn_dict.get("model", "Weather Monitor II")
        self.port = stn_dict.get("port", Station.DEFAULT_PORT)
        self.max_tries = int(stn_dict.get("max_tries", 5))
        self.retry_wait = int(stn_dict.get("retry_wait", 3))
        self.loop_interval = int(stn_dict.get("loop_interval", 1))
        debug_serial = int(stn_dict.get("debug_serial", 0))
        self.last_rain = None  # ?

        loginf("driver version is %s" % DRIVER_VERSION)
        loginf("using serial port %s" % self.port)
        self.station = Station(self.port, debug_serial=debug_serial)
        self.station.open()

    def closePort(self):
        if self.station is not None:
            self.station.close()
            self.station = None

    @property
    def hardware_name(self):
        return self.model

    def getTime(self):
        return self.station.get_time()

    def setTime(self):
        self.station.set_time(int(time.time()))

    def genLoopPackets(self):
        while True:
            packet = {"dateTime": int(time.time() + 0.5), "usUnits": weewx.US}
            readings = self.station.get_readings_with_retry(
                self.max_tries, self.retry_wait
            )
            data = self.station.parse_readings(readings)
            packet.update(data)
            self._augment_packet(packet)
            yield packet
            time.sleep(self.loop_interval)

    def _augment_packet(self, packet):
        packet["rain"] = weewx.wxformulas.calculate_rain(
            packet["rain_total"], self.last_rain
        )
        self.last_rain = packet["rain_total"]


class Station(object):
    DEFAULT_PORT = "/dev/ttyUSB0"

    def __init__(self, port, debug_serial=0):
        self._debug_serial = debug_serial
        self.port = port
        self.baudrate = 2400
        self.timeout = 2  # seconds
        self.serial_port = None
        # Calibrations
        self.tp1cal = 0
        self.tp2cal = 0
        self.rncal = 100
        self.hm1cal = 0
        self.hm2cal = 0
        self.barcal = 0
        self.windcal = 1600

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, _, value, traceback):
        self.close()

    def open(self):
        logdbg("open serial port %s" % self.port)
        self.serial_port = serial.Serial(self.port, self.baudrate, timeout=self.timeout)

    def close(self):
        if self.serial_port is not None:
            logdbg("close serial port %s" % self.port)
            self.serial_port.close()
            self.serial_port = None

    def get_time(self):
        d = self.ReadWRD(6, 1, 0xBE)
        hour = self.fromBCD(ord(d[0:1]))
        min = self.fromBCD(ord(d[1:2]))
        sec = self.fromBCD(ord(d[2:3]))
        d = self.ReadWRD(3, 1, 0xC8)
        day = self.fromBCD(ord(d[0:1]))
        month = ord(d[1:2])
        logdbg("station time: month:%s day:%s %s:%s:%s" % (month, day, hour, min, sec))
        year = datetime.datetime.now().year
        dt = datetime.datetime(year, month, day, hour, min, sec)
        station_time = time.mktime(dt.timetuple())
        return station_time

    def set_time(self, t):
        self.get_time()
        (year, month, day, hour, min, sec, d, d, d) = time.localtime(t)
        logdbg("Attempting to set Station's Time...")
        self.WriteWRD(
            6,
            1,
            0xBE,
            (chr(self.toBCD(hour)) + chr(self.toBCD(min)) + 
               chr(self.toBCD(sec))).encode('utf-8')
        )
        logdbg("set station time = %s:%s:%s" % (hour, min, sec))
        logdbg("Attempting to set Station's Date")
        self.WriteWRD(3, 1, 0xC8, 
            (chr(self.toBCD(day)) + chr(month)).encode('utf-8'))
        logdbg("set station date = mont:%s day:%s" % (month, day))

    def get_acknowledge(self):
        c = self.serial_port.read(1)
        if len(c) == 0:
            raise weewx.WeeWxIOError("Acknowledge returned empty: %s" % c)
        ACK = 6
        if ord(c) != ACK:
            raise weewx.WeeWxIOError("Acknowledge not equal 6: %s" % c)

    def ReadWRD(self, n, bank, addr):
        if bank == 0:
            bankval = 2
        elif bank == 1:
            bankval = 4
        self.serial_port.write(b'WRD' + ((n << 4) | bankval).to_bytes(1,'little') +
           (addr & 0x00ff).to_bytes(1,'little') + b'\r')
        self.get_acknowledge()
        data = self.serial_port.read(int((n + 1) / 2))
        return data

    def ReadByte(self, bank, addr):
        bytes = self.ReadWRD(2, bank, addr)
        rec = struct.unpack("<b", bytes)
        n = rec[0]
        return n

    def ReadWord(self, bank, addr):
        time.sleep(1)  # Time between consecutive calls for console to catch up
        bytes = self.ReadWRD(4, bank, addr)
        rec = struct.unpack("<h", bytes)
        n = rec[0]
        return n

    def WriteWord(self, bank, addr, n):
        time.sleep(1)  # Time between consecutive calls for console to catch up
        bytes = struct.pack("<h", n)
        self.WriteWRD(4, bank, addr, bytes)

    def WriteWRD(self, n, bank, addr, data):
        if bank == 0:
            bankval = 1
        elif bank == 1:
            bankval = 3
        #  Assumes being passed *bytes* not a *string*
        self.serial_port.write(b'WWR' + ((n << 4) | bankval).to_bytes(1,'little') +
           (addr & 0x00ff).to_bytes(1,'little') + data + b'\r')
        self.get_acknowledge()

    def SendSTART(self):
        # self.serial_port.write(b'START' + chr(0x0D))
        self.serial_port.write(b'START\r')
        self.get_acknowledge()

    def SendLOOP(self):
        # self.serial_port.write(b'LOOP' + chr(255) + chr(255) + chr(0x0D))
        self.serial_port.write(b'LOOP\xff\xff\r')
        self.get_acknowledge()

    def GetCalibration(self):
        self.tp1cal = self.ReadWord(1, 0x0152)
        self.tp2cal = self.ReadWord(1, 0x0178)
        self.rncal = self.ReadWord(1, 0x01D6)
        self.hm1cal = 0
        self.hm2cal = self.ReadWord(1, 0x01DA)
        self.barcal = self.ReadWord(1, 0x012C)
        self.windcal = 1600
        logdbg(
            "Station Calibrations: inTemp:%d, outTemp:%d, rain:%d, inHum:%d, outHum:%d, pressure:%d, wind:%d"
            % (
                self.tp1cal,
                self.tp2cal,
                self.rncal,
                self.hm1cal,
                self.hm2cal,
                self.barcal,
                self.windcal,
            )
        )

    def SetCalibration(self, tp1cal, tp2cal, rncal, h2mcal, barcal):
        self.WriteWord(1, 0x0152, tp1cal)
        self.WriteWord(1, 0x0178, tp2cal)
        self.WriteWord(1, 0x01D6, rncal)
        self.WriteWord(1, 0x01DA, h2mcal)
        self.WriteWord(1, 0x012C, barcal)

    def get_readings(self):
        self.SendLOOP()
        c = self.serial_port.read(1)
        if not c:
            raise weewx.WeeWxIOError("Invalid header: %s" % c)
        if ord(c) != 1:
            raise weewx.WeeWxIOError("Invalid header: %s" % c)
        buf = self.serial_port.read(17)
        if len(buf) != 17:
            raise weewx.WeeWxIOError(
                "Invalid Length of Loop response: len %d" % len(buf)
            )
        if weewx.crc16.crc16(buf):  # CRC_checksum
            raise weewx.WeeWxIOError("CRC Checksum error")
        return buf

    def get_readings_with_retry(self, max_tries=5, retry_wait=3):
        for ntries in range(0, max_tries):
            try:
                buf = self.get_readings()
                return buf
            except (serial.serialutil.SerialException, weewx.WeeWxIOError) as e:
                loginf(
                    "Failed attempt %d of %d to get readings: %s"
                    % (ntries + 1, max_tries, e)
                )
                time.sleep(retry_wait)
        else:
            msg = "Max retries (%d) exceeded for readings" % max_tries
            logerr(msg)
            raise weewx.RetriesExceeded(msg)

    def parse_readings(self, raw):
        """Davis Weather Monitor II stations emit data in an 18 bytes package.
        the data is in the following order:
        1 byte  = header ---> 0x01 (checked and removed by get_readings)
        2 bytes = inTemp
        2 bytes = outTemp
        1 byte  = windSpeed
        2 bytes = windDir
        2 bytes = pressure
        1 byte  = inHumidity
        1 byte  = outHumidity
        2 bytes = rain_total ---> rain calibration = 100
        2 bytes = not_used
        2 bytes = CRC_checksum
        """
        if len(raw) != 17:
            warn("wrong size", len(raw))
        buf = struct.unpack("<hhBhhBBhhh", raw)
        data = dict()
        data["windSpeed"] = (buf[2] * 1600.0) / self.windcal  # mph
        data["windDir"] = buf[3]  # compass deg
        data["outTemp"] = (buf[1] + self.tp2cal) / 10.0  # degree_F
        data["rain_total"] = buf[7] / (1.0 * self.rncal)  # inch
        data["pressure"] = (buf[4] + self.barcal) / 1000.0  # inHg
        data["inTemp"] = (buf[0] + self.tp1cal) / 10.0  # degree_F
        data["outHumidity"] = buf[6] + self.hm2cal  # percent
        data["inHumidity"] = buf[5] + self.hm1cal  # percent
        data["rain_total"] = buf[7] / (1.0 * self.rncal)
        logdbg("station data: %s" % data)
        return data

    @staticmethod
    def toBCD(n):
        if n < 0 or n > 99:
            return 0
        n1 = int(n / 10)
        n2 = n % 10
        m = n1 << 4 | n2
        return m

    @staticmethod
    def fromBCD(n):
        n1 = n & 0x0F
        n2 = n >> 4
        return n2 * 10 + n1


class WMIIConfEditor(weewx.drivers.AbstractConfEditor):
    @property
    def default_stanza(self):
        return (
            """
[wmII]
    # This section is for the weewx weather station Davis Weather Monitor II

    # model = station model, e.g., 'Weather Monitor II' or 'Wizard III'
    # [Optional. Default is 'Weather Monitor II']
    model = Weather Monitor II

    # The time (in seconds) between LOOP packets.
    # [Optional. Defaul is 1]
    loop_interval = 1

    # The driver to use:
    driver = user.wmII

    # Serial port such as /dev/ttyS0, /dev/ttyUSB0, or /dev/cua0
    type = serial
    port = %s
"""
            % Station.DEFAULT_PORT
        )

    def prompt_for_settings(self):
        print("Specify the serial port on which the station is connected, for")
        print("example: /dev/ttyUSB0 or /dev/ttyS0 or /dev/cua0.")
        port = self._prompt("port", Station.DEFAULT_PORT)
        return {"port": port}


# define a main entry point for basic testing of the station without weewx
# engine and service overhead.  On a regular Debian insall invoke this as follows:
#
# PYTHONPATH=/usr/share/weewx python /usr/share/weewx/user/wmII.py

if __name__ == "__main__":
    import optparse

    usage = """%prog [options] [--help]"""

    syslog.openlog("Weather Monitor II", syslog.LOG_PID | syslog.LOG_CONS)
    syslog.setlogmask(syslog.LOG_UPTO(syslog.LOG_DEBUG))
    parser = optparse.OptionParser(usage=usage)
    parser.add_option(
        "--version", dest="version", action="store_true", help="display driver version"
    )
    parser.add_option(
        "--debug",
        dest="debug",
        action="store_true",
        help="provide additional debug output in log",
    )
    parser.add_option(
        "--port",
        dest="port",
        metavar="PORT",
        help="serial port to which the station is connected",
        default=Station.DEFAULT_PORT,
    )
    parser.add_option(
        "--gettime",dest="gettime",action="store_true",
        help="Get the current time from the weather station"
    )
    parser.add_option(
        "--settime",dest="settime",action="store_true",
        help="Get, set and then get the current time from the station"
    )

    (options, args) = parser.parse_args()

    if options.version:
        print("Weather Monitor II driver version %s" % DRIVER_VERSION)
        exit(0)

    with Station(options.port, debug_serial=options.debug) as station:
        if options.gettime or options.settime:
           print('Retrieving Station Time (1)')
           stationTime = station.get_time()
           print('Station Time: ' + 
              time.asctime(time.localtime(stationTime)))
           if options.settime:
              print('Setting station Time...')
              station.set_time(stationTime)
              print('Retrieving Station Time (2)')
              stationTime = station.get_time() 
              print('Station Time: ' + 
                 time.asctime(time.localtime(stationTime)))
        while True:
            print(
                time.time(), station.parse_readings(station.get_readings_with_retry())
            )
