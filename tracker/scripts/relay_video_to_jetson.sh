#!/bin/bash
# LAPTOP: forward the Gazebo camera (RTP/H.264 on 127.0.0.1:5600, from the
# ardupilot_gazebo GstCameraPlugin) to the Jetson, which reads it with
#   --camera-source gst --gst-udp-port 5600
# Usage: JETSON_IP=192.168.0.117 scripts/relay_video_to_jetson.sh
JETSON_IP="${JETSON_IP:-192.168.0.117}"
PORT="${VIDEO_PORT:-5600}"
exec gst-launch-1.0 -v udpsrc port="$PORT" \
    caps="application/x-rtp, media=video, encoding-name=H264, payload=96" \
    ! udpsink host="$JETSON_IP" port="$PORT"
