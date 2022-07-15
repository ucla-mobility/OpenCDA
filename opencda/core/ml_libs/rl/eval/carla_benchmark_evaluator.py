import os
import numpy as np
import pandas as pd
from collections import deque
from tqdm import tqdm
from itertools import product
import torch
from typing import Dict, Any, List, Callable, Optional
from tensorboardX import SummaryWriter

from ding.envs import BaseEnvManager
from ding.torch_utils.data_helper import to_tensor
from opencda.core.ml_libs.rl.eval.base_evaluator import BaseEvaluator
from opencda.core.ml_libs.rl.scenario_data.benchmark_suite import ALL_SUITES
from opencda.core.ml_libs.rl.scenario_data.benchmark_utils import get_suites_list, \
                                gather_results, read_pose_txt, get_benchmark_dir


class CarlaBenchmarkEvaluator(BaseEvaluator):
    """
    Evaluator used to evaluate a policy with Carla benchmark evaluation suites. It uses several environments
    in ``EnvManager`` to evaluate policy. For every suites provided by user, evaluator will first find and
    store all available reset params from Benchmark files and store them in a queue such that each reset
    param is evaluated once and only once. The evaluation results are stored in a '.csv' file with reward,
    success and failure status and reset param of the episode. Note: Env manager must run WITHOUT auto reset.

    Arguments
    ----------
    cfg:Dict
        Config dict.
    env:BaseEnvManager
        Env manager used to evaluate.
    policy:Any
        Policy to evaluate. Must have ``forward`` method.
    tb_logger:SummaryWriter, optional
        Tensorboard writter to store values in tensorboard. Defaults to None.
    exp_name: str (optional)
        Name of the experiments. Used to build logger. Defaults to 'default_experiment'.
    instance_name:str(optional)
        Name of the evaluator. Used to build logger. Defaults to 'benchmark_evaluator'.

    Interfaces
    ----------
    reset: None
        Reset the evaluator.
    eval:str
        Run evaluation and return evaluation result.
    close:None
        Close the evaluator and all actors.
    should_eval:bool
        Determine whether to ran evaluation.

    Properties
    ----------
    env:BaseEnvManager
        Env manager with several environments used to evaluate.
    policy:Any
        Policy instance to interact with envs.
    """

    config = dict(
        benchmark_dir=None,
        # dir path to resume&save eval .csv files
        result_dir='',
        # whether to transform obs into tensor manually
        transform_obs=False,
        # num of episodes to eval in a suite
        episodes_per_suite=100,
        # stop value of success rate
        stop_rate=1,
        # whether resume an existing evaluation result in .csv
        resume=False,
        # suite name, can be str or list
        suite='FullTown01-v0',
        # manually set weathers rather than read from suite
        weathers=None,
        seed=0,
        # whether save as .csv file
        save_files=True,
    )

    def __init__(
            self,
            cfg: Dict,
            env: BaseEnvManager,
            policy: Any,
            tb_logger: Optional['SummaryWriter'] = None,  # noqa
            exp_name: Optional[str] = 'default_experiment',
            instance_name: Optional[str] = 'benchmark_evaluator',
    ) -> None:
        super().__init__(cfg, env, policy, tb_logger=tb_logger, exp_name=exp_name, instance_name=instance_name)
        self._benchmark_dir = self._cfg.benchmark_dir
        self._result_dir = self._cfg.result_dir
        self._transform_obs = self._cfg.transform_obs
        self._episodes_per_suite = self._cfg.episodes_per_suite
        self._resume = self._cfg.resume
        if self._benchmark_dir is None:
            self._benchmark_dir = get_benchmark_dir()

        suite = self._cfg.suite
        self._eval_suite_list = get_suites_list(suite)
        self._stop_rate = self._cfg.stop_rate
        self._seed = self._cfg.seed
        self._weathers = self._cfg.weathers
        self._save_files = self._cfg.save_files

        self._last_eval_iter = 0
        self._max_success_rate = 0

    @property
    def env(self) -> BaseEnvManager:
        return self._env_manager

    @env.setter
    def env(self, _env_manager: BaseEnvManager) -> None:
        assert not _env_manager._auto_reset, "auto reset for env manager should be closed!"
        self._end_flag = False
        self._env_manager = _env_manager
        self._env_manager.launch()
        self._env_num = self._env_manager.env_num

    def close(self) -> None:
        """
        Close the collector and the env manager if not closed.
        """
        if self._close_flag:
            return
        self._close_flag = True
        self._env_manager.close()
        if self._tb_logger is not None:
            self._tb_logger.flush()
            self._tb_logger.close()

    def reset(self) -> None:
        """
        Reset evaluator and policies.
        """
        self._policy.reset([i for i in range(self._env_num)])
        self._last_eval_iter = 0
        self._max_success_rate = 0

    def should_eval(self, train_iter: int) -> bool:
        """
        Judge if the training iteration is at frequency value to run evaluation.

        :Arguments:
            - train_iter (int): Current training iteration

        :Returns:
            bool: Whether should run iteration
        """
        if (train_iter - self._last_eval_iter) < self._cfg.eval_freq and train_iter != 0:
            return False
        self._last_eval_iter = train_iter
        return True

    def eval(
            self,
            save_ckpt_fn: Callable = None,
            train_iter: int = -1,
            envstep: int = -1,
            policy_kwargs: Optional[Dict] = None,
            n_episode: Optional[int] = None
    ) -> float:
        """
        Run evaluation with provided policy arguments. It will evaluate all available episodes of the benchmark suite
        unless `episode_per_suite` is set in config.

        :Arguments:
            - save_ckpt_fn (Callable, optional): Function to save ckpt. Will be called if at best performance.
                Defaults to None.
            - train_iter (int, optional): Current training iterations. Defaults to -1.
            - envstep (int, optional): Current env steps. Defaults to -1.
            - policy_kwargs (Dict, optional): Additional arguments in policy forward. Defaults to None.
            - n_episode: (int, optional): Episodes to eval. By default it is set in config.

        :Returns:
            Tuple[bool, float]: Whether reach stop value and success rate.
        """
        if policy_kwargs is None:
            policy_kwargs = dict()
        if n_episode is None:
            n_episode = self._episodes_per_suite
        assert n_episode >= self._env_num, "Episode num must be more than env num!"
        if self._result_dir != '':
            os.makedirs(self._result_dir, exist_ok=True)

        total_time = 0.0
        total_episodes = 0
        success_episodes = 0
        self.reset()

        for suite in self._eval_suite_list:
            args, kwargs = ALL_SUITES[suite]
            assert len(args) == 0
            reset_params = kwargs.copy()
            poses_txt = reset_params.pop('poses_txt')
            weathers = reset_params.pop('weathers')
            suite_name = suite + '_seed%d' % self._seed
            summary_csv = os.path.join(self._result_dir, suite_name + ".csv")
            if os.path.exists(summary_csv) and self._resume:
                summary = pd.read_csv(summary_csv)
            else:
                summary = pd.DataFrame()

            if self._weathers is not None:
                weathers = self._weathers

            pose_pairs = read_pose_txt(self._benchmark_dir, poses_txt)

            episode_queue = deque()
            running_env_params = dict()
            results = []
            running_envs = 0

            for episode, (weather, (start, end)) in enumerate(product(weathers, pose_pairs)):
                if episode >= n_episode:
                    break
                param = reset_params.copy()
                param['start'] = start
                param['end'] = end
                param['weather'] = weather
                if self._resume and len(summary) > 0 and ((summary['start'] == start) & (summary['end'] == end) &
                                                          (summary['weather'] == weather)).any():
                    self._logger.info(
                        '[EVALUATOR] weather: {}, route: ({}, {}) already exist'.format(weather, start, end)
                    )
                    continue
                if running_envs < self._env_num:
                    running_env_params[running_envs] = param
                    running_envs += 1
                else:
                    episode_queue.append(param)

            if not running_env_params:
                self._logger.info("[EVALUATOR] Nothing to eval.")
            else:
                pbar = tqdm(total=len(running_env_params) + len(episode_queue))
                for env_id in running_env_params:
                    self._env_manager.seed({env_id: self._seed})
                self._env_manager.reset(running_env_params)
                with self._timer:
                    while True:
                        obs = self._env_manager.ready_obs
                        for key in obs:
                            if key not in running_env_params:
                                obs.pop(key)
                        if not obs:
                            break
                        if self._transform_obs:
                            obs = to_tensor(obs, dtype=torch.float32)
                        policy_output = self._policy.forward(obs, **policy_kwargs)
                        actions = {env_id: output['action'] for env_id, output in policy_output.items()}
                        timesteps = self._env_manager.step(actions)
                        for i, t in timesteps.items():
                            if t.info.get('abnormal', False):
                                self._policy.reset([i])
                                self._env_manager.reset(reset_params={i: running_env_params[i]})
                                continue
                            if t.done:
                                self._policy.reset([i])
                                result = {
                                    'start': running_env_params[i]['start'],
                                    'end': running_env_params[i]['end'],
                                    'weather': running_env_params[i]['weather'],
                                    'reward': t.info['final_eval_reward'],
                                    'success': t.info['success'],
                                    'collided': t.info['collided'],
                                    'timecost': int(t.info['tick']),
                                }
                                results.append(result)
                                pbar.update(1)
                                if episode_queue:
                                    reset_param = episode_queue.pop()
                                    self._env_manager.reset({i: reset_param})
                                    running_env_params[i] = reset_param
                                else:
                                    running_env_params.pop(i)
                        if self._env_manager.done:
                            break
                duration = self._timer.value
                success_num = 0
                episode_num = 0
                episode_reward = []
                envstep_num = 0
                for result in results:
                    episode_num += 1
                    if result['success']:
                        success_num += 1
                    episode_reward.append(result['reward'])
                    envstep_num += result['timecost']

                if self._save_files:
                    results_pd = pd.DataFrame(results)
                    summary = pd.concat([summary, results_pd])
                    summary.to_csv(summary_csv, index=False)

                info = {
                    'suite': suite,
                    'train_iter': train_iter,
                    'ckpt_name': 'iteration_{}.pth.tar'.format(train_iter),
                    'avg_envstep_per_episode': envstep_num / n_episode,
                    'evaluate_time': duration,
                    'avg_time_per_episode': duration / n_episode,
                    'success_rate': 0 if envstep_num == 0 else success_num / episode_num,
                    'reward_mean': np.mean(episode_reward),
                    'reward_std': np.std(episode_reward),
                }
                if train_iter == -1:
                    info.pop('train_iter')
                    info.pop('ckpt_name')
                elif self._tb_logger is not None:
                    for k, v in info.items():
                        if k in ['train_iter', 'ckpt_name', 'suite']:
                            continue
                        if not np.isscalar(v):
                            continue
                        self._tb_logger.add_scalar('{}_{}_iter/'.format(self._instance_name, suite) + k, v, train_iter)
                        self._tb_logger.add_scalar('{}_{}_step/'.format(self._instance_name, suite) + k, v, envstep)
                self._logger.info(self._logger.get_tabulate_vars_hor(info))

                total_episodes += episode_num
                success_episodes += success_num
                total_time += duration
                pbar.close()

        if self._save_files:
            results = gather_results(self._result_dir)
            print(results)

        success_rate = 0 if total_episodes == 0 else success_episodes / total_episodes
        if success_rate > self._max_success_rate:
            if save_ckpt_fn:
                save_ckpt_fn('ckpt_best.pth.tar')
            self._max_success_rate = success_rate
        stop_flag = success_rate > self._stop_rate and train_iter > 0
        if stop_flag:
            self._logger.info(
                "[EVALUATOR] " +
                "Current success rate: {} is greater than stop rate: {}".format(success_rate, self._stop_rate) +
                ", so the training is converged."
            )
        self._logger.info('[EVALUATOR] Total success: {}/{}.'.format(success_episodes, total_episodes))
        self._logger.info('[EVALUATOR] Total time: {:.3f} hours.'.format(total_time / 3600.0))
        return stop_flag, success_rate