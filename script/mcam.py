#!/usr/bin/python3

import time
import numpy as np
import cv2
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn
import threading
import epics

global pv_imgout

## Device Video
DEV_ID = 0

## Setup epics PVs
print("Setup PVs")
pv_mode = epics.PV("PINK:MCAM:mode", auto_monitor=True)
pv_resolution = epics.PV("PINK:MCAM:resolution", auto_monitor=True)
pv_fps = epics.PV("PINK:MCAM:fps", auto_monitor=True)
pv_acquire = epics.PV("PINK:MCAM:acquire", auto_monitor=True)

pv_imgout = epics.PV("PINK:MCAM:img", auto_monitor=False)
pv_dimx = epics.PV("PINK:MCAM:img_x", auto_monitor=False)
pv_dimy = epics.PV("PINK:MCAM:img_y", auto_monitor=False)
pv_acquire_rbv = epics.PV("PINK:MCAM:acquire_RBV", auto_monitor=False)
pv_mode_rbv = epics.PV("PINK:MCAM:mode_RBV", auto_monitor=False)
pv_resolution_rbv = epics.PV("PINK:MCAM:resolution_RBV", auto_monitor=False)
pv_fps_rbv = epics.PV("PINK:MCAM:fps_RBV", auto_monitor=False)

## camera instant
global cam
print("Setup video capture")
cam = cv2.VideoCapture(DEV_ID)
cam.set(cv2.CAP_PROP_CONVERT_RGB, True)


## main stream handler
def receiver():
    global rawframe
    global rawframeid
    global cam
    global acquire
    rawframe=[[],[]]
    rawframeid=0
    raw=0
    #waits a bit for main thread to start
    time.sleep(5)
    print("Receiver thread running. OK")
    while(True):
        if(acquire):
            ret, raw=cam.read()
            if ret:
                rawframe[(rawframeid+1)%2]=raw
                rawframeid=(rawframeid+1)%1000
            else:
                time.sleep(1)
        else:
            time.sleep(1)

def epics_sender():
    global rawframe
    global rawframeid
    global pv_imgout
    id=0
    time.sleep(5)
    print("EPICS sender thread is running. OK")
    while(True):
        if(id!=rawframeid):
            id=rawframeid
            buf=rawframe[id%2]
            gray = cv2.cvtColor(buf, cv2.COLOR_RGB2GRAY)
            wave = np.reshape(gray, -1)
            pv_imgout.put(wave)
        time.sleep(1)

## start threads
main_receiver = threading.Thread(target=receiver,args=())
epics_sync = threading.Thread(target=epics_sender,args=())

print("Starting threads...")
main_receiver.start()
epics_sync.start()

## main loop
global acquire

mode=-1
resolution=-1
fps=-1
acquire=0
dimx=0
dimy=0

print("\n*** MCAM script running... *** ")
while(True):
    update=False
    ## mode
    try:
        if(mode != int(pv_mode.value) ):
            mode = int(pv_mode.value)
            if(mode==1):
                cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y','U','Y','V'))
            else:
                cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
            update=True
    except:
        print("Error setting up mode")

    ## resolution
    try:
        if(resolution!=int(pv_resolution.value)):
            resolution=int(pv_resolution.value)
            if(resolution==0):
                cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            elif(resolution==1):
                cam.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
                cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
            elif(resolution==2):
                cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            else:
                cam.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
                cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
            update=True
    except:
         print("Error setting up resolution")

    ## rate
    try:
        if(fps!=int(pv_fps.value)):
            fps=int(pv_fps.value)
            if(fps==0):
                cam.set(cv2.CAP_PROP_FPS, 30)
            elif(fps==1):
                cam.set(cv2.CAP_PROP_FPS, 25)
            elif(fps==2):
                cam.set(cv2.CAP_PROP_FPS, 20)
            elif(fps==3):
                cam.set(cv2.CAP_PROP_FPS, 15)
            else:
                cam.set(cv2.CAP_PROP_FPS, 5)
            update=True
    except:
        print("Error setting up framerate")

    ## update
    if(update):
        update=False

        # mode
        param = int(cam.get(cv2.CAP_PROP_FOURCC))
        if(param>1196444237):
            pv_mode_rbv.put(1)
        else:
            pv_mode_rbv.put(0)

        # resolution
        width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
        pv_dimx.put(width)
        pv_dimy.put(height)
        if(width==1280):
            pv_resolution_rbv.put(0)
        elif(width==800):
            pv_resolution_rbv.put(1)
        elif(width==640):
            pv_resolution_rbv.put(2)
        else:
            pv_resolution_rbv.put(3)

        # rate
        rate = int(cam.get(cv2.CAP_PROP_FPS))
        if(rate==30):
            pv_fps_rbv.put(0)
        elif(rate==25):
            pv_fps_rbv.put(1)
        elif(rate==20):
            pv_fps_rbv.put(2)
        elif(rate==15):
            pv_fps_rbv.put(3)
        else:
            pv_fps_rbv.put(4)

    ## acquire
    if(acquire != int(pv_acquire.value)):
        acquire=int(pv_acquire.value)
        pv_acquire_rbv.put(acquire)

    time.sleep(0.5)


print("OK")