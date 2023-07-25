#!/usr/bin/env python3

import time
import cv2
import os

os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp"

gst = "rtsp://127.0.0.1:8554/"

ts = time.time()
cap = cv2.VideoCapture(gst)
if not cap.isOpened() :
    print("capture failed")
    exit()

ret,frame = cap.read()
while ret :
    print('Initialized in {:.2f}'.format(time.time() - ts))
    cv2.imshow('frame',frame)
    ret,frame = cap.read()
    if(cv2.waitKey(1) & 0xFF == ord('q')):
            break;

cap.release()
cv2.destroyAllWindows()
