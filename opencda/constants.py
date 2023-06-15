from enum import Enum


class Profile(Enum):
    DEFAULT = []
    DETECT_YOLO = []
    PREDICTION = ['enable_prediction']
    PREDICTION_OPENCOOD_SINGLE = ['enable_prediction', 'enable_coperception']
    PREDICTION_OPENCOOD_CAV = ['enable_prediction', 'enable_coperception']
