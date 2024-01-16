from picamera import PiCamera
from time import sleep

camera = PiCamera()

camera.start_preview(alpha=192)
sleep(1)
camera.capture('home/gsa202324/Desktop/pic.jpg')
camera.stop_preview()
