import numpy as np
from collections import deque


class StuckDetector(object):
    """
    Stuck detector used to detect vehicle stuck in simulator. It takes speed as input in each tick.

    :Arguments:
        - len_thresh (int, optional): Speed history length to calculate thresh. Defaults to 200.
        - speed_thresh (float, optional): Speed thresh value. Defaults to 0.1.

    :Interfaces:
        - tick, clear
    """

    def __init__(self, len_thresh: int = 200, speed_thresh: float = 0.1) -> None:
        self._speed_queue = deque(maxlen=len_thresh)
        self._len_thresh = len_thresh
        self._speed_thresh = speed_thresh

        self.stuck = False

    def tick(self, speed: float) -> None:
        """
        Update one tick

        :Arguments:
            - speed (float): Current speed
        """
        self._speed_queue.append(speed)
        if len(self._speed_queue) >= self._len_thresh:
            if np.average(self._speed_queue) < self._speed_thresh:
                self.stuck = True
                return
        self.stuck = False

    def clear(self):
        """
        Clear speed history
        """
        self._speed_queue.clear()
