#!/bin/bash

# Nome da imagem Docker
IMAGE_NAME="ros_cbr"

# Construir a imagem
docker build -t $IMAGE_NAME .
