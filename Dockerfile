FROM osrf/ros:humble-desktop

ARG STONEFISH_REF=v1.3
ARG STONEFISH_ROS2_REF=v1.3

ENV DEBIAN_FRONTEND=noninteractive

# --- System deps + pip + transforms3d ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git \
    libglm-dev libsdl2-dev libfreetype6-dev \
    libpcl-dev \
    python3-pip \
    ros-humble-pcl-conversions ros-humble-cv-bridge ros-humble-image-transport \
    && pip3 install transforms3d \
    && rm -rf /var/lib/apt/lists/*

# SDL2 cmake config fix (see Stonefish README)
RUN sed -i 's/-lSDL2 /-lSDL2/' /usr/lib/x86_64-linux-gnu/cmake/SDL2/sdl2-config.cmake || true

# --- Build & install Stonefish (GCC 11 default, compatibile con v1.3 / C++17) ---
RUN git clone https://github.com/patrykcieslak/stonefish.git /opt/stonefish \
    && cd /opt/stonefish && git checkout ${STONEFISH_REF} \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
    && make -j$(nproc) \
    && make install

# --- Workspace setup ---
ENV WS=/root/ros2_ws
RUN mkdir -p $WS/src

# stonefish_ros2 wrapper 
RUN git clone https://github.com/patrykcieslak/stonefish_ros2.git $WS/src/stonefish_ros2 \
    && cd $WS/src/stonefish_ros2 && git checkout ${STONEFISH_ROS2_REF} 

COPY . $WS/src/stonefish_bluerov2

# tf_transformations (non disponibile come apt su Humble, dipende da transforms3d)
RUN git clone https://github.com/DLu/tf_transformations.git $WS/src/tf_transformations

# --- Build workspace ---
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash \
    && cd $WS \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y --skip-keys="pcl python3-transforms3d" \
    && colcon build --symlink-install

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "source $WS/install/setup.bash" >> /root/.bashrc \
    && echo "alias run-stonefish='ros2 launch stonefish_bluerov2 bluerov2_sim.py'" >> /root/.bashrc

COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR $WS
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]



# COME COMPILARE
# cd ~/ros2_ws_ardusub/src/stonefish_bluerov2
# docker build -t stonefish-bluerov2:latest .
#
# COME ESEGUIRE (--network host permette di vedere i topic ROS2 dall'host)
# xhost +local:docker   # abilita X11 per GUI/Stonefish
# docker run -it --rm \
#   --network host \
#   --env DISPLAY=$DISPLAY \
#   --volume /tmp/.X11-unix:/tmp/.X11-unix \
#   --gpus all \
#   --device /dev/dri \
#   --env NVIDIA_DRIVER_CAPABILITIES=all \
#   stonefish-bluerov2:latest
