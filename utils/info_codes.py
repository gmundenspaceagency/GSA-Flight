"""
These are pre-written error and info codes intended to be sent to the ground station
Keys are one-digit HEX-Numbers to minimize data sent to ground
Specific errors might be too long to transmit and therefore will be saved in a separate file locally
There is no code for 'Ground mode' because we can just leave the mode info empty to inicate pi is in ground mode
"""

INFO_CODES = {
    "0": "Error in camera recording",
    "1": "Light sensor 1 not responding",
    "2": "Light sensor 2 or 3 not responding",
    "3": "A/D-Converter not responding",
    "4": "BME280 not responding",
    "5": "Gyroscope not responding",
    "6": "Error in motor script",
    "X": "Guenther not responding",
    "7": "Camera not responding",
    "8": "GPS Module not responding",
    "9": "Unknown error in rotation thread",
    "A": "Unknown error in main loop",
    "B": "Multiplexer not responding",
    "C": "Ascending",
    "D": "Descending",
    "E": "Landed",
    "F": "Single iteration took more than predefined max time",
}