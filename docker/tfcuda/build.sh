#!/bin/bash

export IMAGE_NAME="bdbd"
export CUDA_VERSION="10.1"
export OS="ubuntu20.04"
export TF_VERSION="2.3.0"

# Build the docker images
docker build -t "${IMAGE_NAME}/cuda:${CUDA_VERSION}-base-${OS}" "cuda/dist/${OS}/${CUDA_VERSION}/base"
docker build -t "${IMAGE_NAME}/tf:${TF_VERSION}-base-${OS}" "tensorflow"
