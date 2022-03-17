from easydict import EasyDict
'''
1. [env] defines state, reward, action and episode. It is related to specific learning task. This should be 
   specified by user and should support agent learns the RL objective. 
    --> Define state, reward, action and episode in main config, and pass the settings to this script. 

next:     
*** 2. [env.wrapper] defines the simulation scenario. Simulation benchmarks are listed in ALLSUITE, this config only choose 
   "random (pick benchmark setting randomly)" vs "in order (use benchmark setting one by one)" 
    --> all the benchmark should use OpenCDA to replace
*** Action --> 
    1. behavior
    2. trajectory (Lyft --> output trajectory)
    
    3. input [dict]  ---->  output [dict]  (refer to openCOOD)
    
    
3. [Server] list the Carla port for all envs. 
    --> This should also be defined in the main script and pass to this script.
4. [policy] defines learning parameters/hyper-parameters. 
    --> This should also be defined in the main script and pass to this script.
'''

dqn_config = dict(
    exp_name='dqn21_bev32_buf2e5_lr1e4_bs128_ns3000_update4_train_ft',
    env=dict(
        # Configure action space (i.e., continuous or discrete)
        env_action_space='discrete',
        ACC_LIST = [
            (0, 1),
            (0.25, 0),
            (0.75, 0),
        ],
        STEER_LIST = [
            -0.8,
            -0.5,
            -0.2,
            0,
            0.2,
            0.5,
            0.8,
        ],
        # Collect and eval env num
        collector_env_num=1,
        evaluator_env_num=1,
        simulator=dict(
            town='Town01',
            disable_two_wheels=True,
            verbose=False,
            waypoint_num=32,
            planner=dict(
                type='behavior',
                resolution=1,
            ),
            obs=(
                dict(
                    name='birdview',
                    type='bev',
                    size=[32, 32],
                    pixels_per_meter=1,
                    pixels_ahead_vehicle=14,
                ),
            )
        ),
        col_is_failure=True,
        stuck_is_failure=False,
        ignore_light=True,
        ran_light_is_failure=False,
        off_road_is_failure=True,
        wrong_direction_is_failure=True,
        off_route_is_failure=True,
        off_route_distance=7.5,
        replay_path='./dqn_video',
        visualize=dict(
            type='birdview',
        ),
        manager=dict(
            collect=dict(
                auto_reset=True,
                shared_memory=False,
                context='spawn',
                max_retry=2,
                retry_type='renew',
                step_timeout=120,
                reset_timeout=120,
            ),
            eval=dict()
        ),
        wrapper=dict(
            # Collect and eval suites for training
            collect=dict(suite='train_ft', ),
            eval=dict(suite='FullTown02-v1', ),
        ),
        carla_sim_config = dict(
            suite='FullTown01-v0',
            benchmark_dir=None,
            mode='random',
        )
    ),
    server=[
        # Need to change to you own carla server
        dict(carla_host='localhost', carla_ports=[9000, 9004, 2]),
    ],
    policy=dict(
        cuda=True,
        priority=True,
        nstep=1,
        model=dict(),
        learn=dict(
            batch_size=128,
            learning_rate=0.0001,
            weight_decay=0.0001,
            target_update_freq=100,
            learner=dict(
                hook=dict(
                    # Pre-train model path
                    load_ckpt_before_run='',
                    log_show_after_iter=1000,
                ),
            ),
        ),
        collect=dict(
            n_sample=3000,
            collector=dict(
                collect_print_freq=1000,
                deepcopy_obs=True,
                transform_obs=True,
            ),
        ),
        eval=dict(
            evaluator=dict(
                eval_freq=2000,
                n_episode=5,
                stop_rate=0.7,
                transform_obs=True,
            ),
        ),
        other=dict(
            eps=dict(
                type='exp',
                start=0.95,
                end=0.1,
                decay=10000,
            ),
            replay_buffer=dict(
                replay_buffer_size=200000,
                monitor=dict(
                    sampled_data_attr=dict(
                        print_freq=100,
                    ),
                    periodic_thruput=dict(
                        seconds=120,
                    ),
                ),
            ),
        ),
    ),
)

default_train_config = EasyDict(dqn_config)