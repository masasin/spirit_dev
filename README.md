# SPIRIT (Subimposed Past Image Records Implemented for Teleoperation)

This is my Masters research project.

The thesis (*A UAV Teleoperation System Using Subimposed Past Image Records*) can be found in the [spirit_thesis](https://github.com/masasin/spirit_thesis) repository.

Assumptions:

* Linux system (to share sound)
* NVIDIA video card

Prerequisites:
* Docker
* [nvidia-docker](https://github.com/NVIDIA/nvidia-docker) (to use OpenGL)

After building and running the docker container, run the following commands to enable the necessary packages in the work environment:

    workon spirit
    toggleglobalsitepackages

Note that in Kyoto University, the proxy does not allow building, and until docker allows network sharing, building must happen outside the university.
Alternatively, you can use `docker ps` to find the currently running container, and save the configuration using `docker commit`.
Make sure to update the `run` file to reflect the container you want to run.

If permission errors occur on files created in the docker container, you can run the following command from outside the container in order to make you the owner again:

    sudo chown -R $USER /path/to/folder

In order to run the system:

* [ ] Set up the hardware.
  * [ ] Connect the mocap cameras to the mocap router via ethernet.
  * [ ] Connect the mocap PC to the mocap router via ethernet.
  * [ ] Connect the mocap PC to the operating station via ethernet.
* [ ] Set up the mocap PC. (Windows environment)
  * [ ] Set the motion capture PC's IP address (currently ローカルエリア接続４) to 192.168.0.1.
  * [ ] Start Motive.
  * [ ] Open a recent project file, or:
    * [ ] Perform wanding and ground-plane setup.
    * [ ] Create a rigid body from the markers on the drone.
  * [ ] Open the Rigid Bodies pane.
    * [ ] Set the name to something convenient, such as "drone". (Optional)
    * [ ] Under Advanced, set User Data to "1".
    * [ ] Under Orientation, click "Reset to Current Orientation".
  * [ ] Open the Streaming pane. Ensure all settings are as follows:
    * Broadcast Frame Data: On
    * Local Interface: 192.168.0.1
    * Stream Markers: True
    * Stream Rigid Bodies: True
    * Type: Multicast
    * Command Port: 1510
    * Data Port: 1511
    * Multicast Interface: 239.255.42.99
* [ ] Set up the operating station. (Linux environment)
  * [ ] Create a Wired Ethernet connection with the following settings:
    * Automatically connect to this network when it is available: True
    * IPv4 Method: Manual
    * Address: 192.168.0.2
    * Netmask: 255.255.255.0
    * Gateway: 192.168.0.1
  * [ ] Connect the PS3 controller.
  * [ ] Activate the controller by pressing the central PS button.
  * [ ] Run the docker container.
* [ ] Set up the drone.
    * [ ] Charge the battery if it is not charged. WARNING: Do not leave on charge for too long when full.
    * [ ] Connect the battery. 
    * [ ] Ensure correct startup.
    * [ ] Connect the operating station to the drone WiFi. The password is the string of digits on the SSID.
* [ ] Set up the ROS environment. (Docker container on operating station)
  * [ ] Set appropriate values in the `config/launch_params.yaml` file. (Optional)
  * [ ] Regenerate the launch files: `config/regenerate_launch_files.py`.
* [ ] Launch the system: `roslaunch spirit spirit.launch`.
* [ ] Ensure that:
  * [ ] A live camera feed is displayed. (depends on configuration)
  * [ ] The past image feed is displayed. (depends on configuration)
  * [ ] If the past image feed is displayed, "No data yet" is not displayed.
  * [ ] The battery is high. (The drone will not take off with a low battery.)
* [ ] Test the controller connection:
  * [ ] Identify the Emergency button. (Default is R1.)
  * [ ] Tap the Emergency button. The drone lights should turn red momentarily, and the status should change to Emergency.
* [ ] Launch the drone and fly. Tap the Takeoff button to take off, and the Land button to land. Use the Emergency button in emergencies. 

Troubleshooting:
  * Check the launch configuration file to see if you didn't turn it off by mistake.
  * If the live camera feed is not displayed, check if the operating station is connected to the drone's WiFi.
  * If the drone does not take off, check if the battery is charged.
  * If the past image feed shows "No data yet", check that:
    * the mocap system is connected
    * the mocap system is sending data
    * the mocap system identifies the drone as a rigid body
    * the rigid body's User Data is set to "1"
    * the drone is actually being tracked
