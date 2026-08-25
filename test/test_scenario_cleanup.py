# -*- coding: utf-8 -*-
"""Unit tests for cleanup after partial scenario initialization."""
# License: MIT

import importlib.util
import sys
import types
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import opencda.core.common
import opencda.scenario_testing.evaluations
import opencda.scenario_testing.utils


class InitializationError(Exception):
    """Error raised by a stubbed scenario dependency."""


def make_module(name, **attributes):
    """Create an importable module with the requested attributes."""
    module = types.ModuleType(name)
    for key, value in attributes.items():
        setattr(module, key, value)
    return module


class TestScenarioCleanup(unittest.TestCase):
    """Verify that cleanup preserves initialization failures."""

    scenarios = ("single_town06_carla", "single_2lanefree_carla")

    def load_scenario(self, scenario, cav_world, scenario_manager,
                      evaluation_manager):
        """Load a scenario with its simulator dependencies stubbed."""
        modules = {
            "carla": make_module("carla"),
            "opencda.core.common.cav_world": make_module(
                "opencda.core.common.cav_world", CavWorld=cav_world),
            "opencda.scenario_testing.evaluations.evaluate_manager":
                make_module(
                    "opencda.scenario_testing.evaluations.evaluate_manager",
                    EvaluationManager=evaluation_manager),
            "opencda.scenario_testing.utils.customized_map_api": make_module(
                "opencda.scenario_testing.utils.customized_map_api",
                spawn_helper_2lanefree=object()),
            "opencda.scenario_testing.utils.sim_api": make_module(
                "opencda.scenario_testing.utils.sim_api",
                ScenarioManager=scenario_manager),
            "opencda.scenario_testing.utils.yaml_utils": make_module(
                "opencda.scenario_testing.utils.yaml_utils",
                add_current_time=lambda params: {
                    **params, "current_time": "test-time"}),
        }
        path = (Path(__file__).parents[1] / "opencda" / "scenario_testing" /
                (scenario + ".py"))
        spec = importlib.util.spec_from_file_location(
            "test_" + scenario, path)
        module = importlib.util.module_from_spec(spec)
        with mock.patch.dict(sys.modules, modules):
            spec.loader.exec_module(module)
        return module

    def test_cav_world_failure_is_not_masked(self):
        """Cleanup must not replace the original model-loading error."""
        def fail_cav_world(_):
            raise InitializationError("model loading failed")

        for scenario in self.scenarios:
            with self.subTest(scenario=scenario):
                module = self.load_scenario(
                    scenario, fail_cav_world, mock.Mock(), mock.Mock())
                opt = SimpleNamespace(apply_ml=True,
                                      version="0.9.12",
                                      record=False)

                with self.assertRaisesRegex(InitializationError,
                                            "model loading failed"):
                    module.run_scenario(opt, {})

    def test_partial_scenario_resources_are_released(self):
        """Created managers and vehicles are released after later failure."""
        vehicle = mock.Mock()
        background_vehicle = mock.Mock()
        manager = mock.Mock()
        manager.create_vehicle_manager.return_value = [vehicle]
        manager.create_traffic_carla.return_value = (
            mock.Mock(), [background_vehicle])

        def fail_evaluation_manager(*_, **__):
            raise InitializationError("evaluation setup failed")

        module = self.load_scenario(
            "single_town06_carla",
            mock.Mock(return_value=mock.Mock()),
            mock.Mock(return_value=manager),
            fail_evaluation_manager)
        opt = SimpleNamespace(apply_ml=True,
                              version="0.9.12",
                              record=False)

        with self.assertRaisesRegex(InitializationError,
                                    "evaluation setup failed"):
            module.run_scenario(opt, {})

        manager.close.assert_called_once_with()
        vehicle.destroy.assert_called_once_with()
        background_vehicle.destroy.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
