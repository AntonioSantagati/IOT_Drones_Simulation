FROM osrf/ros:humble-desktop

# Tooling and Python libs to talk to K8s
RUN apt-get update && apt-get install -y \
      python3-pip python3-colcon-common-extensions \
   && rm -rf /var/lib/apt/lists/* \
   && pip3 install --no-cache-dir kubernetes pyyaml

# ROS 2 workspace for interfaces
WORKDIR /ws
COPY application_manager_interfaces /ws/src/application_manager_interfaces

# Build interfaces (generate Python modules into the image)
RUN bash -lc "source /opt/ros/humble/setup.bash && \
              colcon build --packages-select application_manager_interfaces"
RUN apt-get update && apt-get install -y ros-humble-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# Standard DDS env
ENV ROS_DOMAIN_ID=0
