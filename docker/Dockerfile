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

COPY ${PWD}/third_party /home/ubuntu/third_party
WORKDIR /home/ubuntu/third_party
RUN cd nlopt && \
    mkdir build && cd build && \
    cmake .. && \
    make -j8 && \
    make install



WORKDIR /home/ubuntu/third_party 
RUN cd libzmq && \
    mkdir build && cd build && \
    cmake ..  && \
    make -j8 && \
    make install 

WORKDIR /home/ubuntu/third_party 
RUN cd cppzmq && \
    mkdir build && cd build && \
    cmake  -DCPPZMQ_BUILD_TESTS=OFF .. && \
    make -j8 && \
    make install 

WORKDIR /home/ubuntu/third_party 
RUN cd spdlog && \
    mkdir build && cd build && \
    cmake .. && \
    make -j8 && \
    make install 
