FROM tiryoh/ros-desktop-vnc:noetic
LABEL maintainer="summerpaul917@qq.com, Yunkai Xia"
RUN apt-get update -q --fix-missing && \
    apt-get install -y --no-install-recommends --allow-unauthenticated \
    ros-noetic-navigation \
    ros-noetic-mbf-costmap-core \
    ros-noetic-mbf-msgs \
    ros-noetic-costmap-converter \ 
    ros-noetic-uuid-msgs \ 
    tmux \
    libgoogle-glog-dev \
    libgflags-dev \
    libnlopt-dev \
    ros-noetic-libg2o 


RUN apt-get install -y libfmt-dev  terminator

