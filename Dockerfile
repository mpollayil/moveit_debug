FROM ykrobotics/ros2:humble-dev 

ARG DEFAULT_PROGRAMS="/opt/programs"
ENV DEFAULT_PROGRAMS=${DEFAULT_PROGRAMS}

ARG CONTAINER_NAME="localhost"
ENV CONTAINER_NAME=${CONTAINER_NAME}

# Install necessary ros2 packages
RUN sudo apt update -y \
    && sudo apt install -y ros-humble-controller-manager \
    && sudo apt install -y ros-humble-position-controllers \
    && sudo apt install -y ros-humble-gripper-controllers \
    && sudo apt-get install -y ros-humble-joint-trajectory-controller \
    && sudo apt-get install -y ros-humble-joint-state-broadcaster \
    && sudo apt install -y ros-humble-rqt-*   

# Workspace
RUN mkdir -p /home/ros/moveit_debug/src
WORKDIR /home/ros/moveit_debug

# Build packages
COPY *.sh /home/ros/moveit_debug/
RUN ["chmod", "+x", "/home/ros/moveit_debug/build_packages.sh"]
RUN /home/ros/moveit_debug/build_packages.sh

# Finalize
RUN echo "source /opt/ros/humble/setup.bash" >> /home/ros/.bashrc
RUN echo "source /home/ros/install/setup.bash" >> /home/ros/.bashrc
RUN mkdir /opt/programs
RUN ["chown", "-R", "ros:ros", "/opt/programs"]
RUN ["chown", "-R", "ros:ros", "/home/ros/"]
USER ros 
SHELL ["/bin/bash", "--login", "-c"]
EXPOSE 9090
EXPOSE 4001
CMD ["/bin/bash"]
RUN cd /home/ros/moveit_debug
