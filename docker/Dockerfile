FROM tiryoh/ros-desktop-vnc:noetic

# ------ powered by paul xia, summerpaul917@qq.com ------

LABEL maintainer="summerpaul917@qq.com, Yunkai Xia"
RUN apt-get update -q --fix-missing && \
    apt-get install -y --no-install-recommends --allow-unauthenticated \
    ros-noetic-navigation \
    ros-noetic-libg2o \
    ros-noetic-mbf-costmap-core \
    ros-noetic-mbf-msgs \
    ros-noetic-costmap-converter \ 
    libgoogle-glog-dev \
    libgflags-dev \
    libnlopt-dev \
    libboost-all-dev \
	libboost-python-dev \
	libeigen3-dev \
	libgeographic-dev \
	libglfw3-dev \
	libglm-dev \
	libgtest-dev \
	libpugixml-dev \
	python3-catkin-tools \
	libpcap-dev \
	ros-noetic-angles \
	ros-noetic-camera-info-manager \
	ros-noetic-ddynamic-reconfigure \
	ros-noetic-diagnostic-updater \
	ros-noetic-geodesy \
	ros-noetic-jsk-recognition-msgs ros-noetic-visualization-msgs \
	ros-noetic-lanelet2 \
	ros-noetic-nav-msgs \
	ros-noetic-nmea-msgs \
	ros-noetic-tf2-sensor-msgs\
	ros-noetic-navigation*\
	ros-noetic-pointcloud-to-laserscan\
	ros-noetic-map-server


RUN git clone https://github.com/gabime/spdlog.git 
RUN cd spdlog && \
    mkdir build && cd build && \
    cmake .. && \
    make -j8 && \
    make install && \
    ldconfig
RUN cd .. && rm -rf spdlog

