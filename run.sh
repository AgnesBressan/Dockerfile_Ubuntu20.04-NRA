#!/bin/bash

# Permitir que o Docker acesse o servidor X do host
xhost +local:docker

# Nome da imagem Docker
IMAGE_NAME="ros_cbr"

# Nome do contêiner
CONTAINER_NAME="cbr_container"

# Rodar o contêiner com suporte a GPU da NVIDIA
docker run -it --rm \
    --gpus all \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --name $CONTAINER_NAME \
    --env="LIBGL_ALWAYS_SOFTWARE=1" \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --device /dev/dri \
    --privileged \
    --network host \
    $IMAGE_NAME
