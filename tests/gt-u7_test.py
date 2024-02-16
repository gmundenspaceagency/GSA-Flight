import io
import serial
import sys
import logging
import pynmea2

# Konfigurieren des Loggers
logger = logging.getLogger()
logger.addHandler(logging.StreamHandler(sys.stdout))
logger.setLevel(logging.DEBUG)

# Öffnen der seriellen Verbindung
ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)
sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))

# Endlosschleife zum Lesen von NMEA-Nachrichten
while True:
    try:
        line = sio.readline().strip().decode('utf-8')  # Lesen einer Zeile und Dekodieren

        # NMEA-Nachricht parsen
        msg = pynmea2.parse(line)

        # Überprüfen, ob die Nachricht ein GPRMC-Typ ist (optional)
        if isinstance(msg, pynmea2.RMC):
            # Extrahieren der Breiten- und Längengradinformationen
            latitude = msg.latitude
            longitude = msg.longitude

            # Ausgabe der Breiten- und Längengradinformationen
            logger.info("Breitengrad: {}".format(latitude))
            logger.info("Längengrad: {}".format(longitude))

    except serial.SerialException as e:
        logger.error('SerialException: {}'.format(e))
        break
    except UnicodeDecodeError as e:
        logger.error('UnicodeDecodeError: {}'.format(e))
        continue
