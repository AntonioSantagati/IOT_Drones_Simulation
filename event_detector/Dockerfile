FROM osrf/ros:humble-desktop

# ROS tools + CycloneDDS + pip
RUN apt-get update && apt-get install -y \
    python3-pip python3-colcon-common-extensions git \
    ros-humble-rmw-cyclonedds-cpp \
 && rm -rf /var/lib/apt/lists/* \
 && pip3 install --no-cache-dir kubernetes pyyaml

# ROS 2 workspace
WORKDIR /ws

# px4_msgs (aligned to PX4 1.14 / Humble)
RUN git clone -b release/1.14 https://github.com/PX4/px4_msgs.git /ws/src/px4_msgs

# application_manager_interfaces (copied from local dev machine)
COPY application_manager_interfaces /ws/src/application_manager_interfaces

# Event Detector Python node
COPY event_detector_node.py /opt/ed/event_detector_node.py
RUN chmod +x /opt/ed/event_detector_node.py

# Build interfaces (px4_msgs + AM). The Python node itself needs no build.
RUN bash -lc "source /opt/ros/humble/setup.bash && \
              colcon build --packages-select px4_msgs application_manager_interfaces"

# Runtime environment (default to CycloneDDS; can be overridden at deploy time)
ENV ROS_DOMAIN_ID=0
ENV ROS_LOCALHOST_ONLY=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Entry point: source ROS + workspace and launch the node
CMD ["bash","-lc", \
     "source /opt/ros/humble/setup.bash && \
      source /ws/install/setup.bash && \
      python3 /opt/ed/event_detector_node.py"]
