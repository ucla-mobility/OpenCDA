from opencda.scenario_testing.utils import sim_api_replay
from opencda.core.common.cav_world import CavWorld
from opencda.core.common.misc import TOWN_DICTIONARY
import os, time

def run_scenario(opt, scenario_params):
    
    rsu_data_dump = True
    
    dataset_dir = scenario_params['root_dir']
    scene_name = scenario_params['current_time']
    cav_dir = sorted([d for d in os.listdir(os.path.join(dataset_dir,scene_name)) if os.path.isdir(os.path.join(dataset_dir,scene_name,d))])[0]
    timestamps = set(f[:6] for f in os.listdir(os.path.join(dataset_dir,scene_name,cav_dir)))
    timestamps = sorted(list(timestamps))

    # create CAV world
    cav_world = CavWorld(opt.apply_ml)

    # create scenario manager
    scenario_manager = sim_api_replay.ScenarioManagerReplayPly(scenario_params, opt.apply_ml, opt.version,
                                                town=TOWN_DICTIONARY[scene_name], cav_world=cav_world)

    rsu_list = scenario_manager.create_rsu_manager(data_dump=rsu_data_dump)

    try:
        # run steps

        for t in timestamps[:1]:
            
            print(t)

            scenario_manager.tick()

            for rsu in rsu_list:
                if rsu_data_dump:
                    rsu.data_dumper.count = int(t)
            
            # run_step update traffic and dump data
            
            for rsu in rsu_list:
                rsu.update_info()
                rsu.run_step()

    except KeyboardInterrupt:
        pass

    finally:

        time.sleep(8)

        scenario_manager.close()

        if rsu_list is not None:
            for r in rsu_list:
                r.destroy()
