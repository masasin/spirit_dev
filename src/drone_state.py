# -*- coding: utf-8 -*-
# (C) 2016  Jean Nassar
# Released under BSD version 4


class DroneState(object):
    """
    Drone states as defined by `ardrone_autonomy`.

    """
    Emergency = 0
    Inited = 1
    Landed = 2
    Flying = 3
    Hovering = 4
    Test = 5
    TakingOff = 6
    GotoHover = 7
    Landing = 8
    Looping = 9
