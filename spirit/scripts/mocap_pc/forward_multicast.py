#!/usr/bin/env python3
# (C) 2016  Jean Nassar
# Released under BSD version 4
"""
日本語版のドキュメンテーションは英語版の下にある。

Listen to motion capture multicast and forward to an individual computer.

For an unknown reason, I cannot find the required settings to stream position
data from Optitrack's Motive:Tracker motion capture program. The base station,
in ROS (specifically, the `mocap_optitrack` node) requires the data in order to
work.

To fix this, use the following settings in Motive:Tracker's Data Streaming pane
(accessed from the View menu):

    - Broadcast Frame Data is checked.
    - Local Interface is the same as the cameras (ignore the warning)
    - Stream Markers is True
    - Stream Rigid Bodies is True
    - Remote Trigger is False
    - Type is Multicast
    - Command Port is 1510
    - Data Port is 1511
    - Multicast Interface is the same as the `MULTICAST_IP` variable below

The hardware setup is as follows:

    - Connect the cameras and the mocap computer to the same router using
      ethernet
    - Ensure that the base station can talk to the mocap computer (`ping` the
      IP indicated in `ipconfig` for the mocap computer, and `ifconfig` for the
      base station)
    - Set the IP address of `CLIENT_ADDRESS` to the base station IP address.

Once this setup can be done, run this program. If running in Spyder, press F5.
The `mocap_optitrack` node can now run normally.

To end the program, press Ctrl+C.

===============================================================================

モーションキャプチャのマルチキャストを個別のパソコンに転送すること。

理由は分からないが、OptitrackのMotive:Trackerの位置・姿勢データの自動転送の設定は正しく動かない。特に、オペステ
上で実行されるROSの`mocap_optitrack`ノードにはそのデータが必要。

直すには、Motive:TrackerのData Streaming (Viewメニューで見つかる)で次の設定を行う：

    - Broadcast Frame Data はチェックする
    - Local Interface はカメラのインターフェースと同じにする（警告は無視する）
    - Stream Markers は True
    - Stream Rigid Bodies は True
    - Remote Trigger は False
    - Type は Multicast
    - Command Port は 1510
    - Data Port は 1511
    - Multicast Interface は下にある`MULTICAST_IP`変数の値と一致させる

ハードウェアのセットアップは次のように行う:

    - カメラとモーキャプパソコンは同じルーターに有線でつなげる。
    - オペステとモーキャプパソコンの通信が通るかとpingで確かめる。
      IPアドレスを確認するにはオペステで`ifconfig`、モーキャプパソコンで`ipconfig`を実行。
    - `CLIENT_ADDRESS`のIPアドレスは上で確認したオペステのIPアドレスと一致させる。

セットアップが終わったら、本プログラムを実行する。（SpyderならF5ボタンを押す。）
`mocap_optitrack`を正常に実行できるようになる。

本プログラムを終了させるにはCtrl+Cを押す。

"""
import logging
import socket
import struct


def forward_multicast():
    logging.info("Started program")
    logging.debug("Creating sending socket")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        sender.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        logging.debug("Creating receiving socket")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            receiver.bind(SERVER_ADDRESS)

            logging.debug("Connecting to multicast")
            group = socket.inet_aton(MULTICAST_IP)
            request = struct.pack("4sL", group, socket.INADDR_ANY)
            receiver.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                                request)

            logging.info("Forwarding")
            while True:
                try:
                    data, _ = receiver.recvfrom(1024)
                    sender.sendto(data, CLIENT_ADDRESS)
                except KeyboardInterrupt:
                    break
    logging.info("Ending program")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    MULTICAST_IP = "239.255.42.99"  # Must match Multicast Interface in Motive.
    SERVER_ADDRESS = ("", 1511)
    CLIENT_ADDRESS = ("10.249.255.138", 1511)
    forward_multicast()
