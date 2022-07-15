'''
Copyright 2021 OpenDILab. All Rights Reserved:
Description:
'''

import os
import sys

import copy
from abc import abstractmethod
from typing import Any, Dict, Optional
from easydict import EasyDict

from ding.utils.default_helper import deep_merge_dicts
from ding.utils import EasyTimer, build_logger


class BaseEvaluator(object):

    config = dict()

    def __init__(
            self,
            cfg: dict,
            env: Any = None,
            policy: Any = None,
            tb_logger: Optional['SummaryWritter'] = None,  # noqa
            exp_name: Optional[str] = 'default_experiment',
            instance_name: Optional[str] = 'base_evaluator',
    ) -> None:
        if 'cfg_type' not in cfg:
            self._cfg = self.__class__.default_config()
            self._cfg = deep_merge_dicts(self._cfg, cfg)
        else:
            self._cfg = cfg
        self._end_flag = False
        self._exp_name = exp_name
        self._instance_name = instance_name
        self._tb_logger = None
        if tb_logger is not None:
            self._tb_logger = tb_logger
        self._logger, _ = build_logger(
            path='./{}/log/{}'.format(self._exp_name, self._instance_name), name=self._instance_name, need_tb=False
        )
        self._close_flag = False

        self._timer = EasyTimer()
        if env is not None:
            self.env = env
        if policy is not None:
            self.policy = policy

    @property
    def env(self) -> Any:
        return self._env

    @env.setter
    def env(self, _env: Any) -> None:
        self._env = _env

    @property
    def policy(self) -> Any:
        return self._policy

    @policy.setter
    def policy(self, _policy: Any) -> None:
        self._policy = _policy

    @abstractmethod
    def reset(self) -> Any:
        raise NotImplementedError

    @abstractmethod
    def close(self) -> Any:
        raise NotImplementedError

    @abstractmethod
    def eval(self) -> Any:
        raise NotImplementedError

    @classmethod
    def default_config(cls: type) -> EasyDict:
        cfg = EasyDict(cls.config)
        cfg.cfg_type = cls.__name__ + 'Config'
        return copy.deepcopy(cfg)
