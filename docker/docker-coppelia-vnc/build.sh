#!/usr/bin/env bash
chmod +x entrypoint.sh
sudo docker build --platform linux/amd64 -t docker-coppelia-vnc:v1 .
