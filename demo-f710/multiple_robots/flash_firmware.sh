#!/bin/bash

DOCKER_IMAGE=$(yq .services.rosbot.image $(dirname "$0")/compose.rosbot2r.yaml)

docker stop $(docker ps -q)

docker run --rm -it --privileged \
$DOCKER_IMAGE \
/flash-firmware.py /root/firmware.bin