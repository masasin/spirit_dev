#!/usr/bin/env python3
# (C) 2015  Jean Nassar
# Released under BSD version 4
"""
Listen to motion capture multicast and forward to an individual computer.
For an unknown reason, I cannot find the required settings to stream position
data from Optitrack's Motive:Tracker motion capture program. The base station,
in ROS (specifically, the `mocap_optitrack` node, requires that data in order to
work.
To fix this, use the following settings in Motive:Tracker:
    - Broadcast Frame Data is checked.
    - Local Interface is the same as the cameras. (Ignore the warning.)
    - Stream Markers is True
    - Stream Rigid Bodies is True
    - Remote Trigger is False
    - Type is Multicast
    - Command Port is 1510
    - Data Port is 1511
    - Multicast Interface is the same as the `MULTICAST_IP` variable below.
The hardware setup is as follows:
    - Connect the cameras and the mocap computer to the same router using
      ethernet
    - Ensure that the base station can talk to the mocap computer (`ping` the
      IP indicated in `ipconfig` for the mocap computer, and `ifconfig` for the
      base station)
Once this setup can be done, the `mocap_optitrack` node can run normally.
"""
import socket
import struct


def main():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        sender.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            receiver.bind(SERVER_ADDRESS)

            # Connect to multicast
            group = socket.inet_aton(MULTICAST_IP)
            mreq = struct.pack("4sL", group, socket.INADDR_ANY)
            receiver.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                                mreq)

            while True:
                data, _ = receiver.recvfrom(1024)
                sender.sendto(data, CLIENT_ADDRESS)


if __name__ == "__main__":
    MULTICAST_IP = "239.255.42.99"  # Must match Multicast Interface in Motive.
    SERVER_ADDRESS = ("", 1511)
    CLIENT_ADDRESS = ("192.168.0.11", 1511)  # Base station IP address and port
    main()
