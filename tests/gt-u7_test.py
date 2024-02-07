import io
import serial
import sys
import logging

logger = logging.getLogger()
logger.addHandler(logging.StreamHandler(sys.stdout))
logger.setLevel(logging.DEBUG)

ser = serial.Serial(
        port = '/dev/ttyACM0',
        baudrate = 9600,
        parity = serial.PARITY_NONE,
        stopbits = serial.STOPBITS_ONE,
        bytesize = serial.EIGHTBITS,
        timeout = 1
)
sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))

while True:
        try:
                line = sio.readline()

                logger.debug (line)

        except serial.SerialException as e:
                logger.error('SerialException: {}'.format(e))
                break
        except UnicodeDecodeError as e:
                logger.error('UnicodeDecodeError: {}'.format(e))
        continue
