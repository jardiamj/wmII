#!/usr/bin/env python
#
# Copyright 2018 Jardi A. Martinez Jordan <jardiamj@gmail.com>
# See the file LICENSE.txt for your rights.
# Derived from Ultimeter driver.
# Most methods were derived from The Weather Update app by:
#   Scott Hassan <hassan@dotfunk.com>

"""Driver for Davis Weather Monitor II weather station, Should also work with
Wizzard and Perception but hasn't been tested


"""

import serial
import syslog
import time
import datetime
import struct

import weewx.drivers
import weewx.wxformulas
from weewx.units import INHG_PER_MBAR, MILE_PER_KM
from weeutil.weeutil import timestamp_to_string

DRIVER_NAME = 'wmII'
DRIVER_VERSION = '0.2'

def loader(config_dict, _):
    return WMII(**config_dict[DRIVER_NAME])

def confeditor_loader():
    return WMIIConfEditor()


def logmsg(level, msg):
    syslog.syslog(level, 'Weather Monitor II: %s' % msg)

def logdbg(msg):
    logmsg(syslog.LOG_DEBUG, msg)

def loginf(msg):
    logmsg(syslog.LOG_INFO, msg)

def logerr(msg):
    logmsg(syslog.LOG_ERR, msg)

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
        self.model = stn_dict.get('model', 'Weather Monitor II')
        self.port = stn_dict.get('port', Station.DEFAULT_PORT)
        self.max_tries = int(stn_dict.get('max_tries', 5))
        self.retry_wait = int(stn_dict.get('retry_wait', 3))
        self.loop_interval = int(stn_dict.get('loop_interval', 1))
        debug_serial = int(stn_dict.get('debug_serial', 0))
        self.last_rain = None #?

        loginf('driver version is %s' % DRIVER_VERSION)
        loginf('using serial port %s' % self.port)
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
            packet = {'dateTime': int(time.time() + 0.5),
                      'usUnits': weewx.US}
            readings = self.station.get_readings_with_retry(self.max_tries,
                                                            self.retry_wait)
            data = self.station.parse_readings(readings)
            packet.update(data)
            self._augment_packet(packet)
            yield packet
            time.sleep(self.loop_interval)

    def _augment_packet(self, packet):
        packet['rain'] = weewx.wxformulas.calculate_rain(
            packet['rain_total'], self.last_rain)
        self.last_rain = packet['rain_total']

class Station(object):
    DEFAULT_PORT = '/dev/ttyUSB0'

    def __init__(self, port, debug_serial=0):
        self._debug_serial = debug_serial
        self.port = port
        self.baudrate = 2400
        self.timeout = 2 # seconds
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
        self.serial_port = serial.Serial(self.port, self.baudrate,
                                         timeout=self.timeout)

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
        month = ord(d[1])
        logdbg("station time: month:%s day:%s %s:%s:%s" % (month, day, hour,
                                                            min, sec))
        year = datetime.datetime.now().year
        dt = datetime.datetime(year, month, day, hour, min, sec)
        station_time = (time.mktime(dt.timetuple()))
        return station_time

    def set_time(self, t):
        self.get_time()
        (year, month, day, hour, min, sec, d,d,d) = time.localtime(t)
        logdbg("Attempting to set Station's Time...")
        self.WriteWRD(6, 1, 0xBE, chr(self.toBCD(hour)) + chr(self.toBCD(min)) +
                        chr(self.toBCD(sec)))
        logdbg("set station time = %s:%s:%s" % (hour, min, sec))
        logdbg("Attempting to set Station's Date")
        self.WriteWRD(3, 1, 0xC8, chr(self.toBCD(day)) + chr(month))
        logdbg("set station date = mont:%s day:%s" %(month, day))

    def get_acknowledge(self):
        c = self.serial_port.read(1)
        if c == '':
            raise weewx.WeeWxIOError("Acknowledge returned empty: %s" % c)
        ACK = 6
        if ord(c) != ACK:
            raise weewx.WeeWxIOError("Acknowledge not equal 6: %s" % c)

    def ReadWRD(self, n, bank, addr):
		if bank == 0: bankval = 2
		elif bank == 1: bankval = 4
		self.serial_port.write("WRD" + chr((n << 4) | bankval)
                                + chr(addr & 0x00ff) + chr(0xd))
		self.get_acknowledge()
		data = self.serial_port.read((n+1)/2)
		return data

    def ReadByte(self, bank, addr):
        bytes = self.ReadWRD(2, bank, addr)
        rec = struct.unpack('<b', bytes)
        n = rec[0]
        return n

    def ReadWord(self, bank, addr):
        time.sleep(1) #Time between consecutive calls for console to catch up
        bytes = self.ReadWRD(4, bank, addr)
        rec = struct.unpack('<h', bytes)
        n = rec[0]
        return n

    def WriteWord(self, bank, addr, n):
        time.sleep(1) #Time between consecutive calls for console to catch up
        bytes = struct.pack('<h', n)
        self.WriteWRD(4, bank, addr, bytes)

    def WriteWRD(self, n, bank, addr, data):
		if bank == 0: bankval = 1
		elif bank == 1: bankval = 3
		self.serial_port.write("WWR" + chr((bankval) | (n << 4))
                                + chr(addr & 0x00ff) + data + chr(0xd))
		self.get_acknowledge()

    def SendSTART(self):
		self.serial_port.write("START" + chr(0x0d))
		self.get_acknowledge()

    def SendLOOP(self):
        self.serial_port.write("LOOP" + chr(255) + chr(255) + chr(0x0d))
        self.get_acknowledge()

    def GetCalibration(self):
        self.tp1cal = self.ReadWord(1, 0x0152)
        self.tp2cal = self.ReadWord(1, 0x0178)
        self.rncal = self.ReadWord(1, 0x01D6)
        self.hm1cal = 0
        self.hm2cal = self.ReadWord(1, 0x01DA)
        self.barcal = self.ReadWord(1, 0x012C)
        self.windcal = 1600
        logdbg("Station Calibrations: inTemp:%d, outTemp:%d, rain:%d, inHum:%d, outHum:%d, pressure:%d, wind:%d"
        % (self.tp1cal, self.tp2cal, self.rncal, self.hm1cal, self.hm2cal, self.barcal, self.windcal))

    def SetCalibration(self, tp1cal, tp2cal, rncal, h2mcal, barcal):
        self.WriteWord(1, 0x0152, tp1cal)
        self.WriteWord(1, 0x0178, tp2cal)
        self.WriteWord(1, 0x01D6, rncal)
        self.WriteWord(1, 0x01DA, h2mcal)
        self.WriteWord(1, 0x012C, barcal)

    def get_readings(self):
        self.SendLOOP()
        c = self.serial_port.read(1)
        if c == '':
            raise weewx.WeeWxIOError("Invalid header: %s" % c)
        if ord(c) != 1:
            raise weewx.WeeWxIOError("Invalid header: %s" % c)
        buf = self.serial_port.read(17)
        if len(buf) != 17:
            raise weewx.WeeWxIOError("Invalid Lenght of Loop response: len %d"
                                        % len(buf))
        if self.crc16(data=buf, crc=0, table=CRC16_XMODEM_TABLE): #CRC_checksum
            raise weewx.WeeWxIOError("CRC Checksum error")
        return buf

	def SetCalibration(self, tp1cal, tp2cal, rncal, h2mcal, barcal):
		self.WriteWord(1, 0x0152, tp1cal)
		self.WriteWord(1, 0x0178, tp2cal)
		self.WriteWord(1, 0x01D6, rncal)
		self.WriteWord(1, 0x01DA, h2mcal)
		self.WriteWord(1, 0x012C, barcal)

    def get_readings_with_retry(self, max_tries=5, retry_wait=3):
        for ntries in range(0, max_tries):
            try:
                buf = self.get_readings()
                return buf
            except (serial.serialutil.SerialException, weewx.WeeWxIOError), e:
                loginf("Failed attempt %d of %d to get readings: %s" %
                       (ntries + 1, max_tries, e))
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
        2 bytes = CRC_checksum ---> pending implementation
        """
        if len(raw) != 17: warn("wrong size", len(raw))
        buf = struct.unpack('<hhBhhBBhhh', raw)
        data = dict()
        data['windSpeed'] = (buf[2] * 1600.) / self.windcal  # mph
        data['windDir'] = buf[3]  # compass deg
        data['outTemp'] = (buf[1] + self.tp2cal) / 10.  # degree_F
        data['rain_total'] = buf[7] / (1.0 * self.rncal)  # inch
        data['pressure'] = (buf[4] + self.barcal) / 1000.  # inHg
        data['inTemp'] = (buf[0] + self.tp1cal) / 10.  # degree_F
        data['outHumidity'] = buf[6] + self.hm2cal  # percent
        data['inHumidity'] = buf[5] + self.hm1cal  # percent
        data['rain_total'] = buf[7] / (1.0 * self.rncal)
        logdbg("station data: %s" % data)
        return data

    @staticmethod
    def toBCD(n):
    	if n < 0 or n > 99: return 0
    	n1 = n / 10
    	n2 = n % 10
    	m = n1 << 4 | n2
    	return m

    @staticmethod
    def fromBCD(n):
    	n1 = n & 0x0F
    	n2 = n >> 4
    	return n2 * 10 + n1

    @staticmethod
    def crc16(data, crc, table):
        """Calculate CRC16 using the given table.
        `data`      - data for calculating CRC, must be a string
        `crc`       - initial value
        `table`     - table for caclulating CRC (list of 256 integers)
        Return calculated value of CRC
        """
        for byte in data:
            crc = ((crc<<8)&0xff00) ^ table[((crc>>8)&0xff)^ord(byte)]
        return crc & 0xffff

CRC16_XMODEM_TABLE = [
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
        0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
        0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
        0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
        0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
        0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
        0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
        0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
        0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
        0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
        0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
        0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
        0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
        0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
        0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
        0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
        0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
        0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
        0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
        0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
        0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
        0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
        0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
        0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
        0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
        0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
        0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
        0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
        ]

class WMIIConfEditor(weewx.drivers.AbstractConfEditor):
    @property
    def default_stanza(self):
        return """
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
    port = %s
""" % Station.DEFAULT_PORT

    def prompt_for_settings(self):
        print "Specify the serial port on which the station is connected, for"
        print "example: /dev/ttyUSB0 or /dev/ttyS0 or /dev/cua0."
        port = self._prompt('port', Station.DEFAULT_PORT)
        return {'port': port}


# define a main entry point for basic testing of the station without weewx
# engine and service overhead.  invoke this as follows from the weewx root dir:
#
# PYTHONPATH=bin python bin/weewx/drivers/wmII.py

if __name__ == '__main__':
    import optparse

    usage = """%prog [options] [--help]"""

    syslog.openlog('Weather Monitor II', syslog.LOG_PID | syslog.LOG_CONS)
    syslog.setlogmask(syslog.LOG_UPTO(syslog.LOG_DEBUG))
    parser = optparse.OptionParser(usage=usage)
    parser.add_option('--version', dest='version', action='store_true',
                      help='display driver version')
    parser.add_option('--debug', dest='debug', action='store_true',
                      help='provide additional debug output in log')
    parser.add_option('--port', dest='port', metavar='PORT',
                      help='serial port to which the station is connected',
                      default=Station.DEFAULT_PORT)
    (options, args) = parser.parse_args()

    if options.version:
        print "Weather Monitor II driver version %s" % DRIVER_VERSION
        exit(0)

    with Station(options.port, debug_serial=options.debug) as station:
        station.set_logger_mode()
        while True:
            print time.time(), station.get_readings()
