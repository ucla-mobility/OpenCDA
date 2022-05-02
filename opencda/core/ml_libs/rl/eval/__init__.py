from core import SIMULATORS
from .serial_evaluator import SerialEvaluator

if 'carla' in SIMULATORS:
    from .single_carla_evaluator import SingleCarlaEvaluator
#     from .carla_benchmark_evaluator import CarlaBenchmarkEvaluator
