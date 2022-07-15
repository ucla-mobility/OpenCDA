import os
import cv2
import imageio
from pathlib import Path
import numpy as np


def show_image(image, name='test'):
    if image.ndim == 3:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    cv2.imshow(name, image)
    cv2.waitKey(1)


def check_image(image, name='test'):
    if image.ndim == 3:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    cv2.imshow(name, image)
    cv2.waitKey(0)


def save_image(filepath, image):
    if image.ndim == 3:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    cv2.imwrite(filepath, image)


def read_image(filepath):
    if not os.path.exists(filepath):
        raise FileNotFoundError("%s not found!" % filepath)
    image = cv2.imread(filepath)
    if image.ndim == 3:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    if is_grayimage(image):
        image = image[..., :1]
    return image


def is_image(image):
    if isinstance(image, np.ndarray):
        if len(image.shape) == 2:
            return True
        elif len(image.shape) == 3 and image.shape[2] in [1, 3]:
            return True
        return False
    return False


def is_grayimage(image):
    if len(image.shape) == 2:
        return True
    x = abs(image[:, :, 0] - image[:, :, 1])
    y = np.sum(x)
    if y == 0:
        return True
    else:
        return False


def _create_writer(video_path, height, width, fps=10):
    return cv2.VideoWriter('%s.avi' % video_path, cv2.VideoWriter_fourcc(*'XVID'), fps, (width, height))


class VideoMaker(object):
    video = None
    video_path = None

    @classmethod
    def init(cls, save_dir='debug', save_path='video'):
        if cls.video is not None:
            cls.video.release()

        save_dir = Path(save_dir)
        save_dir.mkdir(exist_ok=True, parents=True)

        cls.video = None
        cls.video_path = str(save_dir.joinpath(save_path))

        #cv2.destroyAllWindows()

    @classmethod
    def add(cls, image):
        if image.ndim == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        if cls.video is None:
            cls.video = _create_writer(cls.video_path, image.shape[0], image.shape[1])

        cls.video.write(image)

    @classmethod
    def clear(cls):
        if cls.video is not None:
            cls.video.release()


class GifMaker(object):
    images = dict()

    @classmethod
    def add(cls, key, image):
        if key not in cls.images:
            cls.images[key] = list()

        cls.images[key].append(image.copy())

    @classmethod
    def save(cls, key, save_dir='debug', save_path='test.gif', duration=0.05):
        save_dir = Path(save_dir)

        save_dir.mkdir(exist_ok=True, parents=True)

        imageio.mimsave(str(save_dir.joinpath(save_path)), cls.images[key], 'GIF', duration=duration)

        cls.clear(key)

    @classmethod
    def clear(cls, key=None):
        if key in cls.images:
            cls.images.pop(key)
        else:
            cls.images.clear()


def draw_msra_gaussian(heatmap, center, sigma):
    tmp_size = sigma * 3
    mu_x = int(center[0] + 0.5)
    mu_y = int(center[1] + 0.5)
    w, h = heatmap.shape[0], heatmap.shape[1]
    ul = [int(mu_x - tmp_size), int(mu_y - tmp_size)]
    br = [int(mu_x + tmp_size + 1), int(mu_y + tmp_size + 1)]
    if ul[0] >= h or ul[1] >= w or br[0] < 0 or br[1] < 0:
        return heatmap
    size = 2 * tmp_size + 1
    x = np.arange(0, size, 1, np.float32)
    y = x[:, np.newaxis]
    x0 = y0 = size // 2
    g = np.exp(-((x - x0) ** 2 + (y - y0) ** 2) / (2 * sigma ** 2))
    g_x = max(0, -ul[0]), min(br[0], h) - ul[0]
    g_y = max(0, -ul[1]), min(br[1], w) - ul[1]
    img_x = max(0, ul[0]), min(br[0], h)
    img_y = max(0, ul[1]), min(br[1], w)
    heatmap[
        img_y[0]:img_y[1],
        img_x[0]:img_x[1]] = np.maximum(heatmap[img_y[0]:img_y[1], img_x[0]:img_x[1]], g[g_y[0]:g_y[1], g_x[0]:g_x[1]])
    return heatmap
