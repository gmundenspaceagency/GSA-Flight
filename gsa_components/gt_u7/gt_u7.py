import serial
import pynmea2

# Serielle Verbindung herstellen
ser = serial.Serial('/dev/ttyACM0', 9600)  # Überprüfe die Baudrate entsprechend deiner Anwendung

    while True:
        # Daten vom Port lesen
        data = ser.readline().decode().strip()  # Daten decodieren und unnötige Leerzeichen entfernen
        
        # Daten ausgeben
        print("Empfangene Daten:", data)