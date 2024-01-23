from picamera2.encoders import H264Encoder
from picamera2 import Picamera2, Preview
import time

picam2 = Picamera2()
video_config = picam2.create_video_configuration(main={"size": (192, 108)})
picam2.configure(video_config)
encoder = H264Encoder(bitrate=10000)
output = '/home/gsa202324/Desktop/test.h264'
picam2.start_preview(Preview.QTGL)
picam2.start_recording(encoder, output)
time.sleep(10)
picam2.stop_recording()
picam2.stop_preview()