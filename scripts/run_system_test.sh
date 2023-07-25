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
    --volume="$HOME/.ros:/home/kpv/.ros" \
    --volume="$DIR/..:/home/kpv/catkin_ws/src/smelter-monitoring" \
    -it --rm registry.gitlab.com/kapernikov/umicore/smelter-monitoring/$IMAGE_NAME \
    bash -c "cd /home/kpv/catkin_ws && catkin build && source /home/kpv/catkin_ws/devel/setup.bash && catkin run_tests --no-deps smelter_monitoring -DPYTHON_EXECUTABLE=/usr/bin/python3"
