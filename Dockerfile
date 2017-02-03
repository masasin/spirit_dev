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

WORKDIR /root/catkin_ws/src
RUN git clone https://github.com/ros-drivers/mocap_optitrack.git
RUN git clone https://github.com/bosch-ros-pkg/usb_cam.git

COPY spirit/ /root/catkin_ws/src/spirit

WORKDIR /root/catkin_ws
RUN /ros_entrypoint.sh rosdep install --from-paths src -iy
RUN /ros_entrypoint.sh catkin_make

RUN apt-get install -y python-opengl
RUN apt-get install -y  python-pyqt5.qtopengl
RUN apt-get install -y  python-pyqt5.qtmultimedia
RUN apt-get install -y python-dev
RUN apt-get install -y libxml2-dev
RUN apt-get install -y libxslt1-dev
RUN apt-get install -y zlib1g-dev

# nvidia-docker hooks
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

WORKDIR /root/catkin_ws/src/spirit
RUN apt-get install -y firefox ros-kinetic-rqt-graph
RUN /bin/bash -c "source /usr/local/bin/virtualenvwrapper.sh; mkvirtualenv spirit; pip install bokeh catkin_pkg defusedxml jupyter lxml numpy matplotlib==2.0.0rc2 pandas pygame pyyaml rosbag_pandas roshelper rospkg scipy seaborn tqdm"
RUN mkdir /root/.ros/camera_info
RUN cp config/ardrone_front.yaml /root/.ros/camera_info
RUN cp config/ardrone_bottom.yaml /root/.ros/camera_info
