# Mostafa Ali @ InDro Robotics - 2023
FROM ros:humble
# Ensure that ROS 2 environment is set up
ENV ROS_DISTRO=humble

LABEL org.opencontainers.image.description="InDro indoor image"
LABEL org.opencontainers.image.authors = "Mostafa Ali"
LABEL org.opencontainers.image.vendor = "InDro Robotics"

#RUN which rosdep
SHELL ["/bin/bash", "-c"] 
RUN apt update && \
    apt upgrade -y 
    
RUN apt-get update && apt-get install -y ntpdate
# Install dependencies
RUN apt install -y  python3-pip dmidecode curl ros-humble-rmw-cyclonedds-cpp build-essential git cmake libasio-dev ros-humble-tf2-geometry-msgs \ 
    ros-humble-robot-localization
# Install G-Streamer

RUN apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base \ 
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa \
    gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio

# Install ROS 2 Navigation Specific Packages
RUN apt install -y python3-tk ros-humble-nav2-bringup ros-humble-navigation2 ros-humble-compressed-image-transport python3-colcon-common-extensions ros-humble-cv-bridge

# Install OpenCV
RUN python3 -m pip install opencv-python
RUN curl -fsSL https://www.phidgets.com/downloads/setup_linux | bash - && apt-get install -y libphidget22 libphidget22-dev libphidget22extra

# Install Python 3.11
RUN apt install -y python3.11 wget 

#FROM ghcr.io/indro-robotics/indro_base:latest

#Setting up work directory 
WORKDIR /home/colcon_ws
# setting up ros environment and cloning all the required repo
RUN source /opt/ros/humble/setup.bash && \
    mkdir src && \
    cd src/ && \
    apt update 

RUN pip install pandas
RUN apt update
RUN apt install ros-humble-ros-testing -y
RUN apt install ros-humble-xacro -y
RUN apt install ros-humble-rtabmap-ros -y
#getting the data
################################################################################################################################################################
COPY ./data /home/colcon_ws/src

#Parker mirco-strain-IMU 
################################################################################################################################################################
# --recursive --branch ros2 https://github.com/LORD-MicroStrain/microstrain_inertial.git
WORKDIR /home/colcon_ws
RUN  apt-get install ros-humble-microstrain-inertial-driver -y
RUN  apt-get install ros-humble-microstrain-inertial-rqt -y
RUN pip install setuptools==58.2.0

#Building pakcages
################################################################################################################################################################

RUN rosdep update -y
#RUN source /opt/ros/humble/setup.bash && rosdep install --from-paths src --ignore-src --rosdistro humble -y
RUN source /opt/ros/humble/setup.bash && colcon build

#copying entrypoint file and communication file
WORKDIR /home/colcon_ws
COPY ./ros_entrypoint.sh /home/
COPY ./cyclonedds.xml /home/
WORKDIR /home
RUN chmod +x ros_entrypoint.sh

#robosensce echo
STOPSIGNAL SIGINT
RUN echo "net.core.rmem_max=2147483647" >> /etc/sysctl.d/10-cyclone-max.conf
RUN source /home/colcon_ws/install/setup.bash
ENTRYPOINT [ "/home/ros_entrypoint.sh" ]
CMD [ "bash" ]
