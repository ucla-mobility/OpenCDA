import os
import torch
import numpy as np
from typing import Any, Dict, List, Optional

from ding.torch_utils.data_helper import to_tensor
from ding.utils import build_logger
from tensorboardX import SummaryWriter
from opencda.core.ml_libs.rl.eval.base_evaluator import BaseEvaluator


class SingleCarlaEvaluator(BaseEvaluator):
    """
    Carla evaluator used to evaluate a single environment. It is mainly used to visualize the
    evaluation results. It uses a environment in DI-engine form and can be rendered in the runtime.

    :Arguments:
        - cfg (Dict): Config dict
        - env (Any): Carla env, should be in DI-engine form
        - policy (Any): the policy to pe evaluated
        - exp_name (str, optional): Name of the experiments. Used to build logger. Defaults to 'default_experiment'.
        - instance_name (str, optional): Name of the evaluator. Used to build logger. Defaults to 'single_evaluator'.

    :Interfaces: reset, eval, close

    :Properties:
        - env (BaseDriveEnv): Environment used to evaluate.
        - policy (Any): Policy instance to interact with envs.
    """

    config = dict(
        # whether calling 'render' each step
        render=False,
        # whether transform obs into tensor manually
        transform_obs=False,
    )

    def __init__(
            self,
            cfg: Dict,
            env: Any,
            policy: Any,
            tb_logger: Optional['SummaryWriter'] = None,  # noqa
            exp_name: Optional[str] = 'default_experiment',
            instance_name: Optional[str] = 'single_evaluator',
    ) -> None:
        super().__init__(cfg, env, policy, tb_logger=tb_logger, exp_name=exp_name, instance_name=instance_name)
        self._render = self._cfg.render
        self._transform_obs = self._cfg.transform_obs

    def close(self) -> None:
        """
        Close evaluator. It will close the EnvManager
        """
        self._env.close()

    def reset(self) -> None:
        pass

    def eval(self, reset_param: Dict = None) -> float:
        """
        Running one episode evaluation with provided reset params.

        :Arguments:
            - reset_param (Dict, optional): Reset parameter for environment. Defaults to None.

        :Returns:
            bool: Whether evaluation succeed.
        """
        self._policy.reset([0])
        eval_reward = 0
        success = False
        if reset_param is not None:
            obs = self._env.reset(**reset_param)
        else:
            obs = self._env.reset()

        with self._timer:
            while True:
                if self._render:
                    self._env.render()
                if self._transform_obs:
                    obs = to_tensor(obs, dtype=torch.float32)
                actions = self._policy.forward({0: obs})
                action = actions[0]['action']
                timestep = self._env.step(action)
                obs = timestep.obs
                if timestep.info.get('abnormal', False):
                    # If there is an abnormal timestep, reset all the related variables(including this env).
                    self._policy.reset(**reset_param)
                    action = np.array([0.0, 0.0, 0.0])
                    timestep = self._env.step(action)

                if timestep.done:
                    eval_reward = timestep.info['final_eval_reward']
                    success = timestep.info['success']
                    break

        duration = self._timer.value
        info = {
            'evaluate_time': duration,
            'eval_reward': eval_reward,
            'success': success,
        }
        print(
            "[EVALUATOR] Evaluation ends:\n{}".format(
                '\n'.join(['\t{}: {:.3f}'.format(k, v) for k, v in info.items()])
            )
        )
        print("[EVALUATOR] Evaluate done!")
        return success
