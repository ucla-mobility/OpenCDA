from enum import Enum

mapping = {
    'default': [],
    'prediction_yolo': ['enable_prediction'],
    'prediction_opencood_single': ['enable_prediction', 'enable_coperception'],
    'prediction_opencood_cav': ['enable_prediction', 'enable_coperception']
}


class Profile(Enum):
    DEFAULT = 'default'
    PREDICTION_YOLO = 'prediction_yolo'
    PREDICTION_OPENCOOD_SINGLE = 'prediction_opencood_single'
    PREDICTION_OPENCOOD_CAV = 'prediction_opencood_cav'

    def profiles(self):
        print(f"🚀💯 Experiment mode: {self.value}")
        return mapping[self.value]


suffix = ".xml"
headline_str = """
    ===========================================================================
                            Scenario {}. Profile: {}
                            Town: {}    CAV count: {}
    ============================================================================
"""
