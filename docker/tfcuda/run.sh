#!/bin/bash

# just a demo of how to run a file

docker run \
  -u $(id -u):$(id -g) \
  -it \
  --gpus all \
  --hostname=tensorflow \
  --mount type=bind,source="/home/kent/github/rkent/bdbd",target=/bdbd \
  bdbd/tf:2.3.0-base-ubuntu20.04 \
  /bin/bash
