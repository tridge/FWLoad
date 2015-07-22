#!/bin/bash

[ $# -eq 1 ] || {
    echo "Usage: start_video_receive.sh IPADDRESS"
    exit 1
}

IP="$1"
gst-launch tcpclientsrc host="$IP" port=3389  !  multipartdemux   ! jpegdec  ! autovideosink
