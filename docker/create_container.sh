#!/bin/bash
 sudo docker run -it --platform linux/amd64 --security-opt seccomp=unconfined -p 6080:80 -p 23000:23000 --name coppeliasim_isrlab docker-coppelia-vnc:v1







# sudo docker run --security-opt seccomp=unconfined --shm-size=1g -p 6080:80 -p 23000:23000 -v=$PWD/isrlab:/SharedPrograms:shared --name coppeliasim_isrlab docker-coppelia-vnc:v1
# sudo docker run --security-opt seccomp=unconfined -p 6080:80 -p 23000:23000 -v=$PWD/isrlab:/SharedPrograms:shared --name coppeliasim_isrlab docker-coppelia-vnc:v1




#    sudo docker run                     \
#    -it                             \
#    --gpus='all'                    \
#    --memory="4g"                   \
#    --memory-swap="4g"              \
#    --security-opt                  \
#    seccomp=unconfined              \
#    -p 6080:80                      \
#    -p 23000:23000                  \
#    -v=$PWD/isrlab:/SharedPrograms/ \
#    --name coppeliasim_isrlab2      \
#    docker-coppelia-vnc:v1
