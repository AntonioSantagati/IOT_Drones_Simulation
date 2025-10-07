FROM osrf/ros:humble-desktop

# Build tools + git + pip
RUN apt-get update && apt-get install -y \
    python3-pip python3-colcon-common-extensions git \
    ros-humble-rmw-cyclonedds-cpp \
 && rm -rf /var/lib/apt/lists/*

# Mongo client library
RUN pip3 install --no-cache-dir pymongo

# ROS 2 workspace
WORKDIR /ws

# Bring in the package 
COPY . /ws/src/data_recorder

# Add px4_msgs (branch compatible with PX4 1.14 / Humble)
RUN git clone -b release/1.14 https://github.com/PX4/px4_msgs.git /ws/src/px4_msgs

# Build ROS 2 
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select px4_msgs data_recorder

# DDS env: use CycloneDDS
ENV ROS_DOMAIN_ID=0
ENV ROS_LOCALHOST_ONLY=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Entrypoint: source ROS + workspace and run the node
CMD ["bash","-lc", \
     "source /opt/ros/humble/setup.bash && \
      source /ws/install/setup.bash && \
      ros2 run data_recorder recorder"]
