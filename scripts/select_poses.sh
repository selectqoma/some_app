#!/bin/sh
xhost + # allow connections to X server
IMAGE_NAME=base_focal_x64
DIR=$(cd `dirname $0` && pwd)
docker run\
    --name="kpv-umicore-smelter-monitoring" \
    --network host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --privileged \
    --env USER=kpv \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$DIR/..:/home/kpv/catkin_ws/src/smelter-monitoring" \
    --volume="$DIR/../../smelter-notebooks:/home/kpv/catkin_ws/src/smelter-notebooks" \
    -it --rm registry.gitlab.com/kapernikov/umicore/smelter-monitoring/$IMAGE_NAME \
    /home/kpv/.local/bin/jupyter-notebook /home/kpv/catkin_ws/src/smelter-notebooks/slagsquare/pose_selection/pt_camera_pose_selection_notebook.ipynb
