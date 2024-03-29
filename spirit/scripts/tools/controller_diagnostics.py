#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# (C) 2016  Jean Nassar
# Released under BSD version 4
"""
Implement a controller in order to control the robot.

Provides classes for directional position, button states, and full controller
state, as well as a `Controller` class. The controller type is detected
automatically, and can be used if its button mapping has been previously
registered.

"""
from __future__ import print_function
import logging

import pygame


class Position(object):
    """
    A class containing a controller axis position.

    Parameters
    ----------
    x : float
        The horizontal position of the axis.
    y : float
        The vertical position of the axis.
    inverted : bool, optional
        Whether the direction is y-inverted.

    Attributes
    ----------
    x : float
        The horizontal position of the axis.
    y : float
        The vertical position of the axis.
    inverted : bool
        Whether the direction is y-inverted.

    """
    def __init__(self, x, y, inverted=False):
        self.x = x
        self.y = y
        self.inverted = inverted

    @property
    def direction(self):
        """
        Determine the direction represented by the controller axis position.

        The directions are Up, Down, Left, and Right, and their intermediates.

        Returns
        -------
        str
            "U", "D", "L", and "R" represent Up, Down, Left, and Right
            respectively. Return "U", "D", "L", "R" if the positions are either
            only `x` or only `y`. Otherwise, return "U" or "D", followed by
            "L" or "R", as appropriate. If both the `x` and `y` positions
            are zero, return "none".

        Examples
        --------
        >>> position = Position(0.5, 0.7)
        >>> position.direction
        'UR'
        >>> position = Position(0.9, 0)
        >>> position.direction
        'R'
        >>> position = Position(0, 0)
        >>> position.direction
        'none'

        """
        if self.y > 0:
            vertical = "D" if self.inverted else "U"
        elif self.y < 0:
            vertical = "U" if self.inverted else "D"
        else:
            vertical = ""

        if self.x > 0:
            horizontal = "R"
        elif self.x < 0:
            horizontal = "L"
        else:
            horizontal = ""

        if not vertical and not horizontal:
            direction = "none"
        else:
            direction = vertical + horizontal

        return direction

    def __repr__(self):
        return str((self.x, self.y))

    def __str__(self):
        return "[{x:5.2f}, {y:5.2f}]".format(x=self.x, y=self.y)


class Buttons(object):
    """
    A class representing the button configuration of a controller.

    When registering the mapping for a new controller, please run the
    `get_name()` function to obtain the name to use.

    Parameters
    ----------
    buttons : list
        A list containing the state of each button. 1 if pressed, 0 otherwise.

    Attributes
    ----------
    buttons : list of int
        A list containing the state of each button.
    pressed_buttons: list of str
        A list containing the names of the button that are pressed.
    known_makes : list of str
        A list containing all the makes whose mappings have been registered.

    """
    _button_list = ("□", "✕", "○", "△",      # 0-3
                    "L1", "R1", "L2", "R2",  # 4-7
                    "select", "start",       # 8-9
                    "L3", "R3", "PS",        # 10-12
                    "U", "D", "L", "R")      # 13-16

    _mappings = {"Logitech Logitech RumblePad 2 USB": {},  # No change
                 "Elecom Wireless Gamepad": {1: 3, 2: 1, 3: 2},
                 "WiseGroup.,Ltd JC-PS101U": {  # PS2
                     0: 3, 1: 2, 2: 1, 3:0,  # Buttons
                     8: 9, 9: 8,  # Select, start
                     4: 6, 5: 7, 7: 5, 6: 4,  # Bumpers
                 },
                 "Sony PLAYSTATION(R)3 Controller": {
                     4: 13, 6: 14, 7: 15, 5: 16,  # Dpad
                     10: 4, 11: 5, 8: 6, 9: 7,  # Bumpers
                     0: 8, 3: 9, 16: 12,  # Centre
                     12: 3, 15: 0, 13: 2, 14: 1,  # Buttons
                     1: 10, 2: 11,  # Sticks
                 },
                }

    # Populate the mappings.
    for _make in _mappings:
        for _i in range(len(_button_list)):
            _mappings[_make].setdefault(_i, _i)

    known_makes = list(_mappings.keys())

    def __init__(self, make, buttons):
        self._make = make
        self.buttons = buttons
        self.pressed = [Buttons._button_list[Buttons._mappings[self._make][i]]
                        for i, button in enumerate(self.buttons) if button]

    def is_pressed(self, button):
        """
        Whether a given button is pressed.

        Parameters
        ----------
        button : str
            The name of the button to be checked.

        Returns
        -------
        bool
            Whether the button is pressed.

        """
        return button in self.pressed

    def any_pressed(self, *buttons):
        """
        Whether any given buttons are pressed.

        Parameters
        ----------
        buttons : one or more str
            The name(s) of the buttons to be checked.

        Returns
        -------
        bool
            True if any the buttons are pressed.

        """
        return any(self.is_pressed(button) for button in buttons)

    def all_pressed(self, *buttons):
        """
        Whether all given buttons are pressed.

        Parameters
        ----------
        buttons : one or more str
            The name(s) of the buttons to be checked.

        Returns
        -------
        bool
            True if all the buttons are pressed.

        """
        return all(self.is_pressed(button) for button in buttons)

    def __repr__(self):
        return str(self.buttons)

    def __str__(self):
        return str(self.pressed)


class State(object):
    """
    The state of the object.

    Parameters
    ----------
    dpad : Position
        The position of the dpad.
    lstick : Position
        The position of the left analog stick.
    rstick : Position
        The position of the right analog stick.
    buttons : Buttons
        The state of the buttons.

    Attributes
    ----------
    dpad : Position
        The position of the dpad.
    lstick : Position
        The position of the left analog stick.
    rstick : Position
        The position of the right analog stick.
    buttons : Buttons
        The state of the buttons.

    """
    def __init__(self, dpad, lstick, rstick, buttons):
        self.dpad = dpad
        self.lstick = lstick
        self.rstick = rstick
        self.buttons = buttons

    @property
    def data(self):
        """
        Return the raw data.

        Returns
        -------
        dpad : 2-tuple of float
            The position of the dpad.
        lstick : 2-tuple of float
            The position of the left analog stick.
        rstick : 2-tuple of float
            The position of the right analog stick.
        buttons : list of int
            The list of buttons. 1 if pressed, 0 otherwise.

        """
        return self.dpad, self.lstick, self.rstick, self.buttons

    def __repr__(self):
        return str(self.data)

    def __str__(self):
        """
        A human-readable representation of the state.

        To print on a single line, ensure that the terminal is at least 144
        characters wide, and end your `print` function with a carriage return
        character to go back to the start of the line.

        Returns
        -------
        str
            A string with a maximum length of 144 characters, showing the
            positions of the dpad, and left and right analog sticks; as well
            as a list showing all the buttons that are currently pressed.

        Examples
        --------
        >>> stick = Controller(0, "body")
        >>> try:
        ...     while True:
        ...         print(stick_body.get_state(), end="\\r")
        ... except (KeyboardInterrupt, SystemExit):  # Exit safely.
        ...     Controller.shutdown_all()
        dpad: UR   lstick: [-1.00,  0.00]  rstick: [ 0.12, -0.45]  buttons: []

        """
        return "dpad: {dpad:4}  lstick: {ls}  rstick: {rs}  buttons: {b:75}"\
            .format(dpad=self.dpad.direction,
                    ls=self.lstick, rs=self.rstick,
                    b=str(self.buttons))


class Controller(object):
    """
    A controller to control the robot.

    The controller wraps a pygame Joystick object.

    Parameters
    ----------
    stick_id : int
        The ID of the controller.
    name : str, optional
        The name of the controller.

    Attributes
    ----------
    controller : pygame.joystick.Joystick
        The controller itself.
    stick_id : int
        The ID of the controller.
    make : str
        The make of the controller.
    name : str, optional
        The name of the controller.
    controllers : dict
        A class variable containing all registered controllers. It is used to
        keep track of all controllers to make sure that they all exit safely.

        **Dictionary format :** {stick_id (int): controller (Controller)}

    Raises
    ------
    RuntimeError
        The mapping of the controller buttons is unknown.

    """
    pygame.init()
    controllers = {}

    def __init__(self, stick_id, name=None):
        self._logger = logging.getLogger("controller-{id}".format(id=stick_id))
        self._logger.debug("Initializing controller")
        try:
            self.controller = pygame.joystick.Joystick(stick_id)
        except pygame.error:
            raise
        self.stick_id = stick_id
        self.make = self.controller.get_name()

        if name is not None:
            self.name = name
        else:
            self.name = self.make

        if self.make not in Buttons.known_makes:
            raise RuntimeError("Unknown controller make: {}".format(self.make))

        self.controller.init()

        self._logger.debug("Registering controller")
        Controller.controllers[stick_id] = self

        self._logger.info("Controller initialized")

    @property
    def state(self):
        """
        Read the state of all the inputs of the controller.

        Returns
        -------
        State
            The controller state.

        """
        stick = self.controller
        n_buttons = stick.get_numbuttons()

        self._logger.debug("Syncronizing pygame")
        pygame.event.pump()

        self._logger.debug("Getting state")
        lstick = Position(stick.get_axis(0), stick.get_axis(1), inverted=True)
        rstick = Position(stick.get_axis(2), stick.get_axis(3), inverted=True)
        buttons = Buttons(self.make,
                          [stick.get_button(i) for i in range(n_buttons)])
        try:
            dpad = Position(*stick.get_hat(0))
        except pygame.error:  # PS3 considers the DPAD as buttons.
            pos = [0, 0]
            if "U" in buttons.pressed:
                pos[1] += 1
            if "D" in buttons.pressed:
                pos[1] -= 1
            if "L" in buttons.pressed:
                pos[0] -= 1
            if "R" in buttons.pressed:
                pos[0] += 1
            for direction in ("U", "D", "L", "R"):
                if direction in buttons.pressed:
                    buttons.pressed.remove(direction)
            dpad = Position(*pos)

        return State(dpad, lstick, rstick, buttons)

    def shutdown(self):
        """Safely quits a controller."""
        self._logger.info("Closing controller handler")
        self.controller.quit()
        del Controller.controllers[self.stick_id]
        if not Controller.controllers:
            pygame.quit()

    @classmethod
    def shutdown_all(cls):
        """Safely quit all controllers."""
        logging.info("Closing all controller handlers")
        for controller in list(cls.controllers.values()):
            controller.shutdown()

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.shutdown()

    def __repr__(self):
        return "{name} (ID# {id})".format(name=self.name, id=self.stick_id())

    def __str__(self):
        return self.name


if __name__ == "__main__":
    with Controller(0) as c:
        while True:
            try:
                print(c.state, end="\r")
            except KeyboardInterrupt:
                print()
                print("Ending diagnostics...")
                break

