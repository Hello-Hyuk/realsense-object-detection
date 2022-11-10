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
        ros-noetic-geometry-msgs \
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