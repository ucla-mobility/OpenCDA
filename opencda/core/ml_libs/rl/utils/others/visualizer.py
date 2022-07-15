import os
from typing import Any, Dict, Optional
import cv2
import numpy as np
from easydict import EasyDict
import copy

from ding.utils.default_helper import deep_merge_dicts
from opencda.core.ml_libs.rl.utils.others.image_helper import GifMaker, VideoMaker, show_image, check_image

class Visualizer(object):
    """
    Visualizer is used to visualize sensor data and print info during running.
    It can be used to show a sensor image on screen, save a gif or video file.

    :Arguments:
        - cfg (Dict): Config dict.

    :Interfaces: init, paint, run_visualize, done
    """
    _name = None
    _canvas = None
    _gif_maker = None
    _video_maker = None
    config = dict(
        show_text=True,
        outputs=list(),
        save_dir='',
        frame_skip=0,
    )

    def __init__(self, cfg: Dict) -> None:
        if 'cfg_type' not in cfg:
            self._cfg = self.__class__.default_config()
            self._cfg = deep_merge_dicts(self._cfg, cfg)
        else:
            self._cfg = cfg

        self._text = self._cfg.show_text
        self._outputs = self._cfg.outputs
        self._save_dir = self._cfg.save_dir

        self._count = 0
        self._frame_skip = self._cfg.frame_skip

        if self._save_dir != '':
            os.makedirs(self._save_dir, exist_ok=True)

    def init(self, name: str) -> None:
        """
        Initlaize visualizer with provided name.

        :Arguments:
            - name (str): Name for window or file.
        """
        self._name = name
        if 'gif' in self._outputs:
            self._gif_maker = GifMaker()
        if 'video' in self._outputs:
            self._video_maker = VideoMaker()
            self._video_maker.init(self._save_dir, self._name)

    def paint(self, image: Any, data_dict: Optional[Dict] = None) -> None:
        """
        Paint canvas with observation images and data.

        :Arguments:
            - image: Rendered image.
            - data_dict(Dict, optional): data dict containing information, state, action and so on
        """
        if data_dict is None:
            data_dict = {}
        WHITE = (255, 255, 255)

        self._canvas = np.uint8(image.copy())
        h, w = self._canvas.shape[:2]
        if min(h, w) < 320:
            rate = 320 / min(h, w)
            self._canvas = resize_birdview(self._canvas, rate)

        def _write(text, i, j, canvas=self._canvas, fontsize=0.4):
            rows = [x * (canvas.shape[0] // 15) for x in range(10 + 1)]
            cols = [x * (canvas.shape[1] // 15) for x in range(9 + 1)]
            cv2.putText(canvas, text, (cols[j], rows[i]), cv2.FONT_HERSHEY_SIMPLEX, fontsize, WHITE, 1)

        if self._canvas.shape[0] > 600:
            fontsize = 0.8
        else:
            fontsize = 0.4

        if self._text:
            left_text_pos = 1
            if 'command' in data_dict:
                _command = {
                    -1: 'VOID',
                    1: 'LEFT',
                    2: 'RIGHT',
                    3: 'STRAIGHT',
                    4: 'FOLLOW',
                    5: 'CHANGE LEFT',
                    6: 'CHANGE RIGHT',
                }.get(data_dict['command'], '???')
                _write('Command: ' + _command, left_text_pos, 0, fontsize=fontsize)
                left_text_pos += 1
            if 'agent_state' in data_dict:
                _state = {
                    -1: 'VOID',
                    1: 'NAVIGATING',
                    2: 'BLOCKED_BY_VEHICLE',
                    3: 'BLOCKED_BY_WALKER',
                    4: 'BLOCKED_RED_LIGHT',
                    5: 'BLOCKED_BY_BIKE',
                }.get(data_dict['agent_state'], '???')
                _write('Agent State: ' + _state, left_text_pos, 0, fontsize=fontsize)
                left_text_pos += 1
            if 'speed' in data_dict:
                text = 'Speed: {:.1f}'.format(data_dict['speed'])
                if 'speed_limit' in data_dict:
                    text += '/{:.1f}'.format(data_dict['speed_limit'])
                _write(text, left_text_pos, 0, fontsize=fontsize)
                left_text_pos += 1
            if 'steer' in data_dict and 'throttle' in data_dict and 'brake' in data_dict:
                _write('Steer: {:.2f}'.format(data_dict['steer']), left_text_pos, 0, fontsize=fontsize)
                _write('Throttle: {:.2f}'.format(data_dict['throttle']), left_text_pos + 1, 0, fontsize=fontsize)
                _write('Brake: {:.2f}'.format(data_dict['brake']), left_text_pos + 2, 0, fontsize=fontsize)
                left_text_pos += 3

            right_text_pos = 1
            if 'collided' in data_dict:
                _write('Collided: %s' % data_dict['collided'], right_text_pos, 9, fontsize=fontsize)
                right_text_pos += 1
            if 'total_lights' in data_dict and 'total_lights_ran' in data_dict:
                text = 'Lights Ran: %d/%d' % (data_dict['total_lights_ran'], data_dict['total_lights'])
                _write(text, right_text_pos, 9, fontsize=fontsize)
                right_text_pos += 1
            if 'end_distance' in data_dict:
                text = 'Distance: %.1f' % data_dict['end_distance']
                if 'total_distance' in data_dict:
                    text += '/%.1f' % data_dict['total_distance']
                _write(text, right_text_pos, 9, fontsize=fontsize)
                right_text_pos += 1
            if 'tick' in data_dict:
                text = 'Time: %d' % data_dict['tick']
                if 'end_timeout' in data_dict:
                    text += '/%.1f' % data_dict['end_timeout']
                _write(text, right_text_pos, 9, fontsize=fontsize)
                right_text_pos += 1
            if 'reward' in data_dict:
                _write('Reward: %.1f' % data_dict['reward'], right_text_pos, 9, fontsize=fontsize)
                right_text_pos += 1
            if data_dict.get('stuck', False):
                _write('Stuck!', right_text_pos, 9, fontsize=fontsize)
                right_text_pos += 1
            if data_dict.get('ran_light', False):
                _write('Ran light!', right_text_pos, 9, fontsize=fontsize)
                right_text_pos += 1
            if data_dict.get('off_road', False):
                _write('Off road!', right_text_pos, 9, fontsize=fontsize)
                right_text_pos += 1
            if data_dict.get('wrong_direction', False):
                _write('Wrong direction!', right_text_pos, 9, fontsize=fontsize)
                right_text_pos += 1

    def run_visualize(self) -> None:
        """
        Run one step visualizer. Update file handler or show screen.
        """
        if self._canvas is None:
            return
        self._count += 1
        if self._count > self._frame_skip:
            if 'gif' in self._outputs:
                self._gif_maker.add(self._name, self._canvas)
            if 'video' in self._outputs:
                self._video_maker.add(self._canvas)
            self._count = 0
        if 'show' in self._outputs:
            show_image(self._canvas, name=self._name)

    def done(self) -> None:
        """
        Save file or release file writter, destroy windows.
        """
        if self._gif_maker is not None:
            self._gif_maker.save(self._name, self._save_dir, self._name + '.gif')
            self._gif_maker.clear(self._name)
        if self._video_maker is not None:
            self._video_maker.clear()
        if 'show' in self._outputs:
            cv2.destroyAllWindows()

    @property
    def canvas(self):
        return self._canvas

    @classmethod
    def default_config(cls: type) -> EasyDict:
        cfg = EasyDict(cls.config)
        cfg.cfg_type = cls.__name__ + 'Config'
        return copy.deepcopy(cfg)


def resize_birdview(img, rate):
    assert len(img.shape) == 3
    img_res_list = []
    for i in range(img.shape[2]):
        img_slice = img[..., i]
        img_slice_res = cv2.resize(img_slice, None, fx=rate, fy=rate, interpolation=cv2.INTER_NEAREST)
        img_res_list.append(img_slice_res)
    img_res = np.stack(img_res_list, axis=2)
    return img_res
