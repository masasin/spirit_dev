SPIRIT is the Masters research project of Jean Nassar.

The thesis can be found in the [thesis](https://github.com/masasin/thesis) repository.

Assumptions:

* Linux system (to share sound)
* NVIDIA video card

Prerequisites:
* Docker
* [nvidia-docker](https://github.com/NVIDIA/nvidia-docker) (to use OpenGL)

After building and running the docker container, run the following commands to enable the necessary packages in the work environment:

    workon spirit
    toggleglobalsitepackages

In order to run the system:

* [] Set up the motion capture (mocap) PC.
  * [] Set the motion capture PC's IP address (currently ローカルエリア接続４) to 192.168.0.1.
  * [] Start Motive.
  * [] Open a recent project file, or:
    * [] Perform wanding and ground-plane setup.
    * [] Create a rigid body from the markers on the drone.
  * [] Open the Rigid Bodies pane.
    * [] (Optional) Set the name to something convenient, such as "drone".
    * [] Under Advanced, set User Data to "1".
  * [] Open the Streaming pane. Ensure all settings are as follows:
    * Broadcast Frame Data: On
    * Local Interface: 192.168.0.1
    * Stream Markers: True
    * Stream Rigid Bodies: True
    * Type: Multicast
    * Command Port: 1510
    * Data Port: 1511
    * Multicast Interface: 239.255.42.99
* [] Set up the operating station.
