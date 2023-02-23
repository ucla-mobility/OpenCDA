# -*- coding: utf-8 -*-
"""
keyboard listener: a keyboard listenr to record typeing and can be used for pasue, resume and stop the program using multi-threading.
"""
# Author: Wei Shao <weishao@ucdavis.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

from pynput import keyboard


class KeyListener(object):
    """
    A keyboard listener class to record the states of keys.

    Parameters
    ----------
    None

    Attributes
    ----------
    keys : dict
        Keyboard keys and states
    """
    def __init__(self):
        """
        Initialize a listener instance
        """
        self.keys = {'p': False, 'esc': False}

    def start(self):
        """
        Start keyboard listening

        Parameters
        ----------
        None

        Returns
        -------
        None
        """
        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        listener.start()

    def on_press(self, key):
        """
        Update the dict when a key is pressed

        Parameters
        ----------
        key: A KeyCode represents the description of a key code used by the operating system.

        Returns
        -------
        Update the dict keys
        """
        try:
            print(f'alphanumeric key {key.char} pressed')
            if key.char in self.keys:
                self.keys[key.char] = not self.keys[key.char]
            else:
                self.keys[key] = False
        except AttributeError:
            print(f'special key {key} pressed')

    def on_release(self, key):
        """
        Update the dict keys when a key is released

        Parameters
        ----------
        key: A KeyCode represents the description of a key code used by the operating system.

        Returns
        -------
        Update the dict keys
        """
        if key == keyboard.Key.esc:
            self.keys['esc'] = True



