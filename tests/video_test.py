import picamera
import time

# Initialize the PiCamera
with picamera.PiCamera() as camera:
    camera.resolution = (192, 108)
    camera.start_preview()

    # Start recording
    output = '/home/gsa202324/GSA-Flight/tests/test.h264'
    camera.start_recording(output)

    # Record for 10 seconds
    time.sleep(10)

    # Stop recording
    camera.stop_recording()
    camera.stop_preview()
