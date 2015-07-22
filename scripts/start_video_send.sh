#!/bin/bash

gst-launch v4l2src ! videorate ! 'video/x-raw-yuv,framerate=(fraction)1/2'  ! ffmpegcolorspace  ! jpegenc quality=10 ! multipartmux ! tcpserversink host=0.0.0.0 port=3389
