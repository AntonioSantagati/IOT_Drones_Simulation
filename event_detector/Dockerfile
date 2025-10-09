FROM osrf/ros:humble-desktop

# ROS tools + CycloneDDS + pip
RUN apt-get update && apt-get install -y \
    python3-pip python3-colcon-common-extensions git \
    ros-humble-rmw-cyclonedds-cpp \
 && rm -rf /var/lib/apt/lists/* \
 && pip3 install --no-cache-dir kubernetes pyyaml

# ROS 2 workspace
WORKDIR /ws

# px4_msgs 
RUN git clone -b release/1.14 https://github.com/PX4/px4_msgs.git /ws/src/px4_msgs

# application_manager_interfaces
COPY application_manager_interfaces /ws/src/application_manager_interfaces

# Event Detector Python node
COPY event_detector_node.py /opt/ed/event_detector_node.py
RUN chmod +x /opt/ed/event_detector_node.py

# Build interfaces (px4_msgs + AM) 
RUN bash -lc "source /opt/ros/humble/setup.bash && \
              colcon build --packages-select px4_msgs application_manager_interfaces"

# Runtime environment 
ENV ROS_DOMAIN_ID=0
ENV ROS_LOCALHOST_ONLY=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp


CMD ["bash","-lc", \
     "source /opt/ros/humble/setup.bash && \
      source /ws/install/setup.bash && \
      python3 /opt/ed/event_detector_node.py"]
