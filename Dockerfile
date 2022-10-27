FROM nvcr.io/nvidia/tensorrt:22.02-py3
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y sudo && apt-get upgrade -y &&\
    # Install build tools, build dependencies and python
    apt-get install -y \
        lsb-release \
	    python3-pip \
        build-essential \
        cmake \
        git \
        wget \
        unzip \
        yasm \
        pkg-config \
        software-properties-common \
        ## Python
        python3-dev \
        python3-numpy &&\
    # for tensorrt
    pip install onnx \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean all

# # install opencv
# RUN cd /opt/ &&\
#     # Download and unzip OpenCV and opencv_contrib and delte zip files
#     wget https://github.com/opencv/opencv/archive/$OPENCV_VERSION.zip &&\
#     unzip $OPENCV_VERSION.zip &&\
#     rm $OPENCV_VERSION.zip &&\
#     wget https://github.com/opencv/opencv_contrib/archive/$OPENCV_VERSION.zip &&\
#     unzip ${OPENCV_VERSION}.zip &&\
#     rm ${OPENCV_VERSION}.zip &&\
#     # Create build folder and switch to it
#     mkdir /opt/opencv-${OPENCV_VERSION}/build && cd /opt/opencv-${OPENCV_VERSION}/build &&\
#     # Cmake configure
#     cmake \
#         -DOPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib-${OPENCV_VERSION}/modules \
#         -DWITH_CUDA=ON \
#         -DCUDA_ARCH_BIN=7.5,8.0,8.6 \
#         -DCMAKE_BUILD_TYPE=RELEASE \
#         # Install path will be /usr/local/lib (lib is implicit)
#         -DCMAKE_INSTALL_PREFIX=/usr/local \
#         .. &&\
#     # Make
#     make -j"$(nproc)" && \
#     # Install to /usr/local/lib
#     make install && \
#     ldconfig &&\
#     # Remove OpenCV sources and build folder
#     rm -rf /opt/opencv-${OPENCV_VERSION} && rm -rf /opt/opencv_contrib-${OPENCV_VERSION}

# install ros-noetic-desktop-full
# RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' &&\
#     curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - &&\
#     sudo apt update &&\
#     sudo apt install -y \
#         ros-noetic-desktop-full \
#         ros-noetic-common-msgs \
#         ros-noetic-sensor-msgs \
#         ros-noetic-cv-bridge \
#         python3-rosdep \
#         python3-rosinstall \
#         python3-rosinstall-generator \
#         python3-wstool &&\
#     source /opt/ros/noetic/setup.bash && echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && source ~/.bashrc &&\
#     sudo rosdep init && rosdep update 

# Install ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update
RUN apt-get install -y --no-install-recommends \
        ros-noetic-desktop-full \
        python3-rosdep \
        ros-noetic-common-msgs \
        ros-noetic-sensor-msgs \
        ros-noetic-vision-msgs \
        ros-noetic-cv-bridge \
        python3-rosdep \
    && rm -rf /var/lib/apt/lists/* 
RUN rosdep init \
 && rosdep fix-permissions \
 && rosdep update
RUN source /opt/ros/noetic/setup.bash && echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && source ~/.bashrc


# Install RealsensSDK
RUN sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
RUN apt-get install -y \
        librealsense2-dkms \
        librealsense2-utils \
        librealsense2-dev \
        librealsense2-dbg \
        ros-noetic-ddynamic-reconfigure
RUN pip install pyrealsense2
RUN sudo apt-get update && sudo apt-get upgrade

# tensorrt
RUN echo "./src/tensorrt_demos/tensorrt_make.sh" >> ~/.bashrc

RUN mkdir /workspace/wego_ws
WORKDIR /workspace/wego_ws