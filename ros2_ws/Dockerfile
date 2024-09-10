FROM osrf/ros:humble-desktop-full

ARG ROS_DISTRO=humble
ARG USERNAME=rosdev
ARG UID=1000
ARG GID=$UID

RUN apt-get update -q \
    && apt-get upgrade -q -y \
    && apt-get install -y --no-install-recommends \
    software-properties-common \
    python3-pip \
    nano \
    xauth \
    sudo \
    v4l-utils \
    python3-opencv \
    wget

# Mesa Drivers for GPU
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    mesa-utils \
    libegl1-mesa \
    libgbm1 \
    libvulkan1

ENV XDG_RUNTIME_DIR=/tmp

# Install Gazebo
RUN apt-get install lsb-release gnupg -y \
    && curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" |  tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y ignition-fortress

RUN apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Create a non-root user
RUN groupadd -g $GID $USERNAME \
    && useradd -lm -u $UID -g $USERNAME -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER $USERNAME

# Create a ROS 2 workspace
RUN mkdir -p /home/$USERNAME/ros2_ws
WORKDIR /home/$USERNAME/ros2_ws

# Copy entrypoint script
COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN sudo chmod +x /ros_entrypoint.sh


ENV NV_CUDA_LIB_VERSION 12.5.1-1

ENV NV_NVTX_VERSION 12.5.82-1
ENV NV_LIBNPP_VERSION 12.3.0.159-1
ENV NV_LIBNPP_PACKAGE libnpp-12-5=${NV_LIBNPP_VERSION}
ENV NV_LIBCUSPARSE_VERSION 12.5.1.3-1

ENV NV_LIBCUBLAS_PACKAGE_NAME libcublas-12-5
ENV NV_LIBCUBLAS_VERSION 12.5.3.2-1
ENV NV_LIBCUBLAS_PACKAGE ${NV_LIBCUBLAS_PACKAGE_NAME}=${NV_LIBCUBLAS_VERSION}

ENV NV_LIBNCCL_PACKAGE_NAME libnccl2
ENV NV_LIBNCCL_PACKAGE_VERSION 2.22.3-1
ENV NCCL_VERSION 2.22.3-1
ENV NV_LIBNCCL_PACKAGE ${NV_LIBNCCL_PACKAGE_NAME}=${NV_LIBNCCL_PACKAGE_VERSION}+cuda12.5


RUN  apt-get update && apt-get install -y --no-install-recommends \
    cuda-libraries-12-5=${NV_CUDA_LIB_VERSION} \
    ${NV_LIBNPP_PACKAGE} \
    cuda-nvtx-12-5=${NV_NVTX_VERSION} \
    libcusparse-12-5=${NV_LIBCUSPARSE_VERSION} \
    ${NV_LIBCUBLAS_PACKAGE} \
    ${NV_LIBNCCL_PACKAGE} 

# Keep apt from auto upgrading the cublas and nccl packages. See https://gitlab.com/nvidia/container-images/cuda/-/issues/88
RUN apt-mark hold ${NV_LIBCUBLAS_PACKAGE_NAME} ${NV_LIBNCCL_PACKAGE_NAME}


ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
