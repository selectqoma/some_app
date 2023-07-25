FROM ros:noetic-ros-base-focal

# create kapernikov user, add to required groups and delete password authentication
RUN adduser --gecos "Kapernikov User" --disabled-password kpv
RUN usermod -aG video kpv
RUN usermod -aG sudo kpv
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# setup ros
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
RUN echo 'Etc/UTC' > /etc/timezone && \
  ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime
RUN apt-get update && apt-get install -q -y tzdata && rm -rf /var/lib/apt/lists/*
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
# TODO apt-get upgrade seems to break Qt based apps like rqt_image_view plugin
# RUN apt-get update && apt-get upgrade -y && apt-get install -q -y \
RUN apt-get update && apt-get install -q -y \
  apt-utils \
  cron \
  curl \
  libfreetype6-dev \
  git \
  git-lfs \
  iputils-ping \
  libgstreamer-plugins-bad1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  libgstreamer-plugins-good1.0-dev \
  libgstrtspserver-1.0-dev \
  libimage-exiftool-perl \
  logrotate \
  net-tools \
  python3-catkin-tools \
  python3-osrf-pycommon \
  python3-opencv \
  python3-pip \
  python3-rosdep \
  python3-tk \
  python3-yaml \
  vim \
  vlc \
  && rm -rf /var/lib/apt/lists/*

# Install dependencies from debian packages
ADD ./dependencies/*deb /tmp/
RUN dpkg -i --force-overwrite /tmp/*deb
RUN rm -rf /tmp/*

# setup git-lfs
RUN git config --global lfs.batch false
RUN git lfs install --skip-repo
RUN ls -lah /home/kpv

# Clone the packages
USER kpv
RUN mkdir -p /home/kpv/catkin_ws/src/
WORKDIR /home/kpv/catkin_ws/src/
RUN git clone --depth=1 --branch=melodic-devel https://github.com/machinekoder/ros_pytest.git
RUN git clone --depth=1 --branch=noetic-devel https://github.com/kapernikov/executive_smach_visualization.git
RUN git clone --depth=1 --branch=deployment https://gitlab+deploy-token-186755:FsFbdkj5DKud1u-9AneQ@gitlab.com/Kapernikov/Umicore/smelter-datasets-and-models.git

# Install python dependencies via pip
ADD ./src/ip_camera_driver/requirements.txt /tmp/requirements_ip_camera_driver.txt
ADD ./src/slagsquare_pot_state_server/requirements.txt /tmp/requirements_pot_state_server.txt
ADD ./src/smelter_tests/requirements.txt /tmp/requirements_tests.txt
ADD ./src/slagsquare_pot_instance_segmentation/requirements.txt /tmp/requirements_pot_detector.txt
RUN pip3 install --no-cache-dir \
  git+https://github.com/Kapernikov/Mask_RCNN.git \
  -r /tmp/requirements_ip_camera_driver.txt \
  -r /tmp/requirements_pot_state_server.txt \
  -r /tmp/requirements_tests.txt \
  -r /tmp/requirements_pot_detector.txt

# Add the main repository and remove .git folders to reduce the size of the image
RUN mkdir -p /home/kpv/catkin_ws/src/smelter-monitoring
ADD --chown=kpv:kpv . /home/kpv/catkin_ws/src/smelter-monitoring
RUN bash -c "rm -rf /home/kpv/catkin_ws/src/smelter-monitoring/{dependencies,guides}"
RUN bash -c "( find /home/kpv/catkin_ws/src/ -type d -name .git ) | xargs -d '\n' rm -rf"

# Install ros dependencies via rosdep
USER root
RUN apt-get update \
  && DEBIAN_FRONTEND=noninteractive rosdep install --from-paths /home/kpv/catkin_ws/src --ignore-src --rosdistro noetic -y -r \
  && rm -rf /var/lib/apt/lists/*
RUN rm -rf /tmp/*

# Setup catkin workspace
USER kpv
WORKDIR /home/kpv/catkin_ws
RUN /bin/bash -c "catkin init"
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"
RUN bash -c "echo 'source /opt/ros/noetic/setup.bash' >> /home/kpv/.bashrc"
RUN bash -c "echo 'source /home/kpv/catkin_ws/devel/setup.bash' >> /home/kpv/.bashrc"

# Add log management
RUN mkdir -p /home/kpv/.ros/bags/{slagsquare,cascade}
RUN mkdir -p /home/kpv/.ros/fff
RUN bash -c "crontab /home/kpv/catkin_ws/src/smelter-monitoring/logrotation/logrotate_cronjob"

USER kpv
WORKDIR /home/kpv
ENTRYPOINT ["/home/kpv/catkin_ws/src/smelter-monitoring/docker/base_focal_x64/ros_entrypoint.sh"]
CMD ["bash"]
