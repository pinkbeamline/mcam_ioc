#!/usr/bin/python3

import time
import numpy as np
import cv2
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn
from enum import Enum
import threading
import epics
import psutil


global pv_imgout
global pv_frameid
global pv_acquire
global pv_acquire_stream
global jpgframe
global jpgframeid
global state
global rawframe
global rawframeid

############################################################################
### Classes and functions
############################################################################

## class enumeration for cam states
class CamState(Enum):
    OFF=0
    CONFIG=1
    RUNNING=2
    RELEASE=3
    QUIT=4

## main stream handler
def receiver(pv_mode, pv_mode_rbv, pv_resolution, pv_resolution_rbv, pv_fps, pv_fps_rbv, pv_status):
    global rawframe
    global rawframeid
    global state

    DEV_ID = 0
    raw=0

    print("[THREAD] Receiver started")

    num_resolutions=9
    cam_resolutions=[(2592,1944),(2048,1536),(1920,1080),(1600,1200),(1280,960),(1280,720),(1020,768),(640,480),(320,240)]
    num_fps=4
    cam_fps=[30, 15, 7.5, 3]

    while(state!=CamState.QUIT):
        if(state==CamState.CONFIG):
            try:
                cam=cv2.VideoCapture(DEV_ID)
                cam.set(cv2.CAP_PROP_CONVERT_RGB, True)

                ## mode
                if(pv_mode.value==1):
                    cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y','U','Y','V'))
                else:
                    cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
                param = int(cam.get(cv2.CAP_PROP_FOURCC))
                if(param>1196444237):
                    pv_mode_rbv.put(1)
                else:
                    pv_mode_rbv.put(0)

                ## resolution
                resolution=int(pv_resolution.value)
                if(resolution>=num_resolutions):
                    resolution=num_resolutions-1
                res_pair=cam_resolutions[resolution]

                width=res_pair[0]
                height=res_pair[1]
                cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
                pv_dimx.put(width)
                pv_dimy.put(height)
                pv_resolution_rbv.put(resolution)

                ## rate
                fps=int(pv_fps.value)
                if(fps<num_fps):
                    setfps=cam_fps[fps]
                else:
                    setfps=cam_fps[0]
                cam.set(cv2.CAP_PROP_FPS, setfps)
                rate = cam.get(cv2.CAP_PROP_FPS)
                pv_fps_rbv.put(rate)

                ## Config OK
                state=CamState.RUNNING
                pv_status.put("Running")

            except:
                state=CamState.RELEASE
                pv_status.put("[Err] Error during cam setup. Releasing resources...")

        while(state==CamState.RUNNING):
            ret, raw=cam.read()
            if ret:
                rawframe[(rawframeid+1)%2]=raw
                rawframeid=(rawframeid+1)%1000
            else:
                print("[Err][receiver]: Could not grab a frame")
                time.sleep(1)

        if(state==CamState.RELEASE):
            cam.release()
            state=CamState.OFF
            pv_status.put("Idle")

        time.sleep(1)

    print("[THREAD] Receiver ended")


def epics_sender(pv_imgout, pv_frameid, pv_acquire):
    global rawframe
    global rawframeid
    global state

    id=0
    counter=0
    ts=0

    print("[THREAD] EPICS sender thread is running")
    while(state!=CamState.QUIT):
        if(int(pv_acquire.value)):
            if( (id!=rawframeid) and (time.time()-ts>1.0) ):
                ts=time.time()
                id=rawframeid
                buf=rawframe[id%2]
                gray = cv2.cvtColor(buf, cv2.COLOR_RGB2GRAY)
                wave = np.reshape(gray, -1)
                pv_imgout.put(wave)
                counter=(counter+1)%1000000
                pv_frameid.put(counter)
            else:
                time.sleep(0.1)
        time.sleep(1)
    print("[THREAD] EPICS sender thread ended")

def jcompressor(pv_acquire_stream, pv_acquire):
    global rawframe
    global rawframeid
    global jpgframe
    global jpgframeid
    global state

    id=0
    parameters = [cv2.IMWRITE_JPEG_QUALITY, 90]
    print("[THREAD] JPG compressor thread is running")
    while(state!=CamState.QUIT):
        if(int(pv_acquire_stream.value)):
            if(id != rawframeid):
                id=rawframeid
                raw=rawframe[id%2]
                #buf=cv2.cvtColor(raw, cv2.COLOR_RGB2GRAY)
                result, jpgframe = cv2.imencode('.jpg', raw, parameters)
                jpgframeid=id
        else:
            time.sleep(0.01)
    print("[THREAD] JPG compressor thread has ended")


### RPI Monitor
def rpimonitor(pv_cputemp, pv_cpuusage, pv_netup, pv_netdl):
    global state
    tpast=time.time()
    pastnetsent=0
    pastnetrecv=0

    print("[THREAD] rpimonitor started")

    while(state != CamState.QUIT):
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
            print("Err[rpimonitor] Could not update RPI usage informartion")
        time.sleep(1)

    print("[THREAD] rpimonitor ended")

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


######################################################################
### Main loop
######################################################################
state = CamState.OFF

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
pv_status = epics.PV("PINK:MCAM:status", auto_monitor=False)

## Threads
thd_stats = threading.Thread(target=rpimonitor,args=(pv_cputemp, pv_cpuusage, pv_netup, pv_netdl))
thd_stats.start()

thd_rx = threading.Thread(target=receiver,args=(pv_mode, pv_mode_rbv, pv_resolution, pv_resolution_rbv, pv_fps, pv_fps_rbv, pv_status) )
thd_rx.start()

thd_epics = threading.Thread(target=epics_sender,args=(pv_imgout, pv_frameid, pv_acquire))
thd_epics.start()

thd_jpg = threading.Thread(target=jcompressor,args=(pv_acquire_stream, pv_acquire) )
thd_jpg.start()

webhandler = ThreadedHTTPServer(('', 8080), Handler)
webserver = threading.Thread(target=webhandler.serve_forever,args=())
webserver.start()

## set up global variables
rawframe=[[],[]]
rawframeid=0

## State loop handler
cid=0
print("[THREAD] Main thread running...")
pv_status.put("Idle")
try:
    while(True):
        if(state==CamState.OFF):
           if(pv_acquire.value or pv_acquire_stream.value):
               state=CamState.CONFIG
               pv_status.put("Setting up camera...")
               #thd_rx.start()
        elif(state==CamState.CONFIG):
           #state=CamState.RUNNING
           #pv_status.put("Running")
           pass
        elif(state==CamState.RUNNING):
           if( (pv_acquire.value==0) and (pv_acquire_stream.value==0) ):
               state=CamState.RELEASE
               pv_status.put("Releasing resources...")
        elif(state==CamState.RELEASE):
           pass
           #thd_rx.join()
           #state=CamState.OFF
           #pv_status.put("Idle")
        else:
           #state=CamState.OFF
           pass
        time.sleep(1)
except:
    print("[THREAD] Main thread canceled")
    state=CamState.QUIT

print("Waiting for threads to quit...")

webserver.shutdown()

thd_stats.join()
thd_rx.join()
thd_jpg.join()
webserver.join()

print("Bye!")

