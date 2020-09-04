#!/usr/bin/python3

import time
import numpy as np
import cv2
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn
import threading
import epics
import psutil

global pv_imgout
global pv_frameid
global pv_acquire
global pv_acquire_stream
global jpgframe
global jpgframeid

## Device Video
DEV_ID = 0

class Handler(BaseHTTPRequestHandler):

    def do_GET(self):
        global jpgframe
        global jpgframeid
        id = 0
        self.send_response(200)
        self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
        self.end_headers()
        #message =  threading.currentThread().getName()
        try:
            while(True):
                if id != jpgframeid:
                    id=jpgframeid
                    buf = bytearray(jpgframe)
                    length = len(buf)
                    self.wfile.write("--jpgboundary\r\n".encode("utf-8"))
                    self.send_header('Content-type', 'image/jpeg')
                    self.send_header('Content-length', str(length))
                    self.end_headers()
                    self.wfile.write(buf)
                else:
                    time.sleep(0.01)
        except:
            pass
        return

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in a separate thread."""

## Setup epics PVs
print("Setup PVs")
pv_mode = epics.PV("PINK:MCAM:mode", auto_monitor=True)
pv_resolution = epics.PV("PINK:MCAM:resolution", auto_monitor=True)
pv_fps = epics.PV("PINK:MCAM:fps", auto_monitor=True)
pv_acquire = epics.PV("PINK:MCAM:acquire", auto_monitor=True)
pv_acquire_stream = epics.PV("PINK:MCAM:acquire_stream", auto_monitor=True)

pv_imgout = epics.PV("PINK:MCAM:img", auto_monitor=False)
pv_dimx = epics.PV("PINK:MCAM:img_x", auto_monitor=False)
pv_dimy = epics.PV("PINK:MCAM:img_y", auto_monitor=False)
pv_acquire_rbv = epics.PV("PINK:MCAM:acquire_RBV", auto_monitor=False)
pv_acquire_stream_rbv = epics.PV("PINK:MCAM:acquire_stream_RBV", auto_monitor=False)
pv_mode_rbv = epics.PV("PINK:MCAM:mode_RBV", auto_monitor=False)
pv_resolution_rbv = epics.PV("PINK:MCAM:resolution_RBV", auto_monitor=False)
pv_fps_rbv = epics.PV("PINK:MCAM:fps_RBV", auto_monitor=False)
pv_frameid = epics.PV("PINK:MCAM:frameid", auto_monitor=False)
pv_cputemp = epics.PV("PINK:MCAM:cputemp", auto_monitor=False)
pv_cpuusage = epics.PV("PINK:MCAM:cpuusage", auto_monitor=False)
pv_netup = epics.PV("PINK:MCAM:netup", auto_monitor=False)
pv_netdl = epics.PV("PINK:MCAM:netdl", auto_monitor=False)

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
    global pv_acquire
    global pv_acquire_stream
    rawframe=[[],[]]
    rawframeid=0
    raw=0
    #waits a bit for main thread to start
    time.sleep(5)
    print("Receiver thread running. OK")
    while(True):
        if( (int(pv_acquire.value))or(int(pv_acquire_stream.value)) ):
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
    global pv_frameid
    global pv_acquire
    id=0
    counter=0
    time.sleep(5)
    print("EPICS sender thread is running. OK")
    while(True):
        if(int(pv_acquire.value)):
            if(id!=rawframeid):
                id=rawframeid
                buf=rawframe[id%2]
                gray = cv2.cvtColor(buf, cv2.COLOR_RGB2GRAY)
                wave = np.reshape(gray, -1)
                pv_imgout.put(wave)
                counter=(counter+1)%1000000
                pv_frameid.put(counter)
        time.sleep(1)

def jcompressor():
    global rawframe
    global rawframeid
    global jpgframe
    global jpgframeid
    global pv_acquire_stream
    id=0
    parameters = [cv2.IMWRITE_JPEG_QUALITY, 90]
    time.sleep(5)
    print("JPG compressor thread is running. OK")
    while(True):
        if(int(pv_acquire_stream.value)):
            if(id != rawframeid):
                id=rawframeid
                raw=rawframe[id%2]
                #buf=cv2.cvtColor(raw, cv2.COLOR_RGB2GRAY)
                result, jpgframe = cv2.imencode('.jpg', raw, parameters)
                jpgframeid=id
        else:
            time.sleep(0.01)

## start threads
main_receiver = threading.Thread(target=receiver,args=())
epics_sync = threading.Thread(target=epics_sender,args=())
img_compressor = threading.Thread(target=jcompressor,args=())

webhandler = ThreadedHTTPServer(('', 8080), Handler)
webserver = threading.Thread(target=webhandler.serve_forever,args=())

print("Starting threads...")
main_receiver.start()
epics_sync.start()
img_compressor.start()
webserver.start()

## main loop
global acquire

mode=-1
resolution=-1
fps=-1
acquire=0
dimx=0
dimy=0
loopid=0
cputemp=0
cpuusage=0
tnow=1
tpast=0
pastnetsent=0
pastnetrecv=0

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
    #if(acquire != int(pv_acquire.value)):
    #    acquire=int(pv_acquire.value)
    #    pv_acquire_rbv.put(acquire)


    ## stats
    loopid=(loopid+1)%1000
    if(loopid%4==0):
        try:
            with open('/sys/class/thermal/thermal_zone0/temp','r') as f:
                cputemp = float(f.read())/1000
            cpuusage=psutil.cpu_percent()

            pv_cputemp.put(cputemp)
            pv_cpuusage.put(cpuusage)

            tnow=time.time()
            netstat=psutil.net_io_counters(pernic=True)

            dt = tnow-tpast
            tpast=tnow

            netup = ((netstat['eth0'].bytes_sent-pastnetsent)/dt)*(8e-6)
            netdl = ((netstat['eth0'].bytes_recv-pastnetrecv)/dt)*(8e-6)

            pastnetsent=netstat['eth0'].bytes_sent
            pastnetrecv=netstat['eth0'].bytes_recv

            pv_netup.put(netup)
            pv_netdl.put(netdl)

        except:
            pass
    time.sleep(0.5)

print("OK")
