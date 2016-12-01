FROM ros:kinetic-robot

MAINTAINER Jean Nassar <jeannassar5@gmail.com>

RUN apt-get update
RUN apt-get install -y sudo vim python-pip
RUN pip install --upgrade pip
RUN pip install virtualenv virtualenvwrapper
ENV WORKON_HOME /root/.virtualenvs
RUN mkdir -p $WORKON_HOME
RUN echo "source /usr/local/bin/virtualenvwrapper.sh" >> /root/.bashrc

RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN /ros_entrypoint.sh catkin_init_workspace
WORKDIR /root/catkin_ws
RUN /ros_entrypoint.sh catkin_make

RUN echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

RUN apt-get install -y ros-kinetic-ardrone-autonomy
RUN apt-get install -y ros-kinetic-image-proc
RUN apt-get install -y ros-kinetic-usb-cam

WORKDIR /root/catkin_ws/src
# RUN git clone https://github.com/AutonomyLab/ardrone_autonomy.git -b indigo-devel
RUN git clone https://github.com/ros-drivers/mocap_optitrack.git

COPY spirit/ /root/catkin_ws/src/spirit

WORKDIR /root/catkin_ws
RUN /ros_entrypoint.sh rosdep install --from-paths src -iy
RUN /ros_entrypoint.sh catkin_make

WORKDIR /root/catkin_ws/src/spirit
RUN /bin/bash -c "source /usr/local/bin/virtualenvwrapper.sh; mkvirtualenv spirit; pip install catkin_pkg defusedxml jupyter lxml numpy pygame pyyaml rospkg tqdm"
