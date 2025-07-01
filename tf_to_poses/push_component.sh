#!/bin/sh


export DOCKER_CLI_EXPERIMENTAL=enabled


docker buildx create --name nimbus-builder 
docker buildx use nimbus-builder
docker run --privileged --rm tonistiigi/binfmt --install all
docker buildx inspect --bootstrap



docker buildx build  --no-cache --platform linux/arm64,linux/amd64 -t cognimbus/tf_to_poses:humble --push .


