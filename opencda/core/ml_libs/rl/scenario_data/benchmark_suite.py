VERSION = '099'

WEATHER_1 = [1, 3, 6, 8]  # train
WEATHER_2 = [4, 14]  # test
WEATHER_3 = [10, 14]
WEATHER_4 = [1, 8, 14]  # validate

ALL_SUITES = dict()


def _add(suite_name, *args, **kwargs):
    assert suite_name not in ALL_SUITES, '%s is already registered!' % suite_name

    town = None

    if 'Town01' in suite_name:
        town = 'Town01'
    elif 'Town02' in suite_name:
        town = 'Town02'
    elif 'Town04' in suite_name:
        town = 'Town04'
    else:
        raise Exception('No town specified: %s.' % suite_name)

    benchmark = 'carla100' if 'NoCrash' in suite_name else 'corl2017'
    suite = None

    if 'Turn' in suite_name:
        suite = 'turn'
    elif 'Straight' in suite_name:
        suite = 'straight'
    elif 'ChangeLane' in suite_name:
        suite = 'changelane'
    elif 'Full' in suite_name:
        suite = 'full'
    elif 'NoCrash' in suite_name:
        suite = 'nocrash'
    else:
        raise Exception('No suite specified: %s.' % suite_name)

    kwargs['town'] = town
    kwargs['poses_txt'] = '%s/%s/%s_%s.txt' % (benchmark, VERSION, suite, town)
    kwargs['col_is_failure'] = 'NoCrash' in suite_name

    ALL_SUITES[suite_name] = (args, kwargs)


# ============= Register Suites ============ ##
# _add('DebugTown01-v0', DebugSuite, n_vehicles=10, viz_camera=True)
# _add('FullTown01-v0', n_vehicles=0, viz_camera=True)
# _add('FullTown02-v0', n_vehicles=0, viz_camera=True)

# data collection town; no respawn to prevent missing frames
_add('FullTown01-v0', n_vehicles=0, weathers=WEATHER_1, respawn_peds=False)
# Train town, train weathers.
_add('FullTown01-v1', n_vehicles=0, weathers=WEATHER_1)
_add('StraightTown01-v1', n_vehicles=0, weathers=WEATHER_1)
_add('TurnTown01-v1', n_vehicles=0, weathers=WEATHER_1)
_add('FullTown04-v1', n_vehicles=0, weathers=WEATHER_1)
_add('StraightTown04-v1', n_vehicles=0, weathers=WEATHER_1)
_add('ChangeLaneTown04-v1', n_vehicles=0, weathers=WEATHER_1)

# Train town, test weathers.
_add('FullTown01-v2', n_vehicles=0, weathers=WEATHER_2)
_add('StraightTown01-v2', n_vehicles=0, weathers=WEATHER_2)
_add('TurnTown01-v2', n_vehicles=0, weathers=WEATHER_2)
_add('FullTown04-v2', n_vehicles=0, weathers=WEATHER_2)
_add('StraightTown04-v2', n_vehicles=0, weathers=WEATHER_2)
_add('ChangeLaneTown04-v2', n_vehicles=0, weathers=WEATHER_2)

# Train town, more vehicles
_add('FullTown01-v3', n_vehicles=20, n_pedestrians=50, weathers=WEATHER_1)
_add('FullTown01-v4', n_vehicles=20, n_pedestrians=50, weathers=WEATHER_2)
_add('FullTown04-v3', n_vehicles=20, n_pedestrians=50, weathers=WEATHER_1)
_add('FullTown04-v4', n_vehicles=20, n_pedestrians=50, weathers=WEATHER_2)
# No ped versions
_add('FullTown01-v3-np', n_vehicles=20, n_pedestrians=0, weathers=WEATHER_1)
_add('FullTown01-v4-np', n_vehicles=20, n_pedestrians=0, weathers=WEATHER_2)
_add('FullTown04-v3-np', n_vehicles=20, n_pedestrians=0, weathers=WEATHER_1)
_add('FullTown04-v4-np', n_vehicles=20, n_pedestrians=0, weathers=WEATHER_2)

# Test town, train weathers.
_add('FullTown02-v1', n_vehicles=0, weathers=WEATHER_1)
_add('StraightTown02-v1', n_vehicles=0, weathers=WEATHER_1)
_add('TurnTown02-v1', n_vehicles=0, weathers=WEATHER_1)

# Test town, test weathers.
_add('FullTown02-v2', n_vehicles=0, weathers=WEATHER_2)
_add('StraightTown02-v2', n_vehicles=0, weathers=WEATHER_2)
_add('TurnTown02-v2', n_vehicles=0, weathers=WEATHER_2)

# Test town, more vehicles.
_add('FullTown02-v3', n_vehicles=15, n_pedestrians=50, weathers=WEATHER_1)
_add('FullTown02-v4', n_vehicles=15, n_pedestrians=50, weathers=WEATHER_2)
# No ped versions
_add('FullTown02-v3-np', n_vehicles=15, n_pedestrians=0, weathers=WEATHER_1)
_add('FullTown02-v4-np', n_vehicles=15, n_pedestrians=0, weathers=WEATHER_2)

_add('NoCrashTown01-v1', n_vehicles=0, disable_two_wheels=True, weathers=WEATHER_1)
_add('NoCrashTown01-v2', n_vehicles=0, disable_two_wheels=True, weathers=WEATHER_3)
_add('NoCrashTown01-v3', n_vehicles=20, disable_two_wheels=True, n_pedestrians=50, weathers=WEATHER_1)
_add('NoCrashTown01-v4', n_vehicles=20, disable_two_wheels=True, n_pedestrians=50, weathers=WEATHER_3)
_add('NoCrashTown01-v5', n_vehicles=100, disable_two_wheels=True, n_pedestrians=250, weathers=WEATHER_1)
_add('NoCrashTown01-v6', n_vehicles=100, disable_two_wheels=True, n_pedestrians=250, weathers=WEATHER_3)
# No ped versions
_add('NoCrashTown01-v3-np', n_vehicles=20, disable_two_wheels=True, n_pedestrians=0, weathers=WEATHER_1)
_add('NoCrashTown01-v4-np', n_vehicles=20, disable_two_wheels=True, n_pedestrians=0, weathers=WEATHER_3)
_add('NoCrashTown01-v5-np', n_vehicles=100, disable_two_wheels=True, n_pedestrians=0, weathers=WEATHER_1)
_add('NoCrashTown01-v6-np', n_vehicles=100, disable_two_wheels=True, n_pedestrians=0, weathers=WEATHER_3)

_add('NoCrashTown02-v1', n_vehicles=0, disable_two_wheels=True, weathers=WEATHER_1)
_add('NoCrashTown02-v2', n_vehicles=0, disable_two_wheels=True, weathers=WEATHER_3)
_add('NoCrashTown02-v3', n_vehicles=15, disable_two_wheels=True, n_pedestrians=50, weathers=WEATHER_1)
_add('NoCrashTown02-v4', n_vehicles=15, disable_two_wheels=True, n_pedestrians=50, weathers=WEATHER_3)
_add('NoCrashTown02-v5', n_vehicles=70, disable_two_wheels=True, n_pedestrians=150, weathers=WEATHER_1)
_add('NoCrashTown02-v6', n_vehicles=70, disable_two_wheels=True, n_pedestrians=150, weathers=WEATHER_3)
# No ped versions
_add('NoCrashTown02-v3-np', n_vehicles=15, disable_two_wheels=True, n_pedestrians=0, weathers=WEATHER_1)
_add('NoCrashTown02-v4-np', n_vehicles=15, disable_two_wheels=True, n_pedestrians=0, weathers=WEATHER_3)
_add('NoCrashTown02-v5-np', n_vehicles=70, disable_two_wheels=True, n_pedestrians=0, weathers=WEATHER_1)
_add('NoCrashTown02-v6-np', n_vehicles=70, disable_two_wheels=True, n_pedestrians=0, weathers=WEATHER_3)

# Demo
_add('NoCrashTown01-v7', n_vehicles=100, n_pedestrians=250, weathers=WEATHER_1)
_add('NoCrashTown01-v8', n_vehicles=100, n_pedestrians=250, weathers=WEATHER_2)
_add('NoCrashTown02-v7', n_vehicles=70, n_pedestrians=150, weathers=WEATHER_1)
_add('NoCrashTown02-v8', n_vehicles=70, n_pedestrians=150, weathers=WEATHER_2)

# Weather primes.
_add('FullTown01-v5', n_vehicles=0, weathers=WEATHER_4)
_add('FullTown01-v6', n_vehicles=20, weathers=WEATHER_4)
_add('StraightTown01-v3', n_vehicles=0, weathers=WEATHER_4)
_add('TurnTown01-v3', n_vehicles=0, weathers=WEATHER_4)
_add('FullTown04-v5', n_vehicles=0, weathers=WEATHER_4)
_add('FullTown04-v6', n_vehicles=20, weathers=WEATHER_4)
_add('StraightTown04-v3', n_vehicles=0, weathers=WEATHER_4)
_add('ChangeLaneTown04-v3', n_vehicles=0, weathers=WEATHER_4)

_add('FullTown02-v5', n_vehicles=0, weathers=WEATHER_4)
_add('FullTown02-v6', n_vehicles=15, weathers=WEATHER_4)
_add('StraightTown02-v3', n_vehicles=0, weathers=WEATHER_4)
_add('TurnTown02-v3', n_vehicles=0, weathers=WEATHER_4)

# Random
_add('NoCrashTown01_noweather_empty', weathers=[1], n_vehicles=0)
_add('NoCrashTown01_noweather_regular', weathers=[1], n_vehicles=20, n_pedestrians=50)
_add('NoCrashTown01_noweather_dense', weathers=[1], n_vehicles=100, n_pedestrians=250)

_add('NoCrashTown02_noweather_empty', weathers=[1], n_vehicles=0)
_add('NoCrashTown02_noweather_regular', weathers=[1], n_vehicles=15, n_pedestrians=50)
_add('NoCrashTown02_noweather_dense', weathers=[1], n_vehicles=70, n_pedestrians=200)

_add('StraightTown01-noweather', n_vehicles=0, weathers=[1])
_add('TurnTown01-noweather', n_vehicles=0, weathers=[1])
_add('FullTown01-noweather-nav', n_vehicles=0, weathers=[1])
_add('FullTown01-noweather', n_vehicles=20, weathers=[1])

_add('StraightTown02-noweather', n_vehicles=0, weathers=[1])
_add('TurnTown02-noweather', n_vehicles=0, weathers=[1])
_add('FullTown02-noweather-nav', n_vehicles=0, weathers=[1])
_add('FullTown02-noweather', n_vehicles=15, weathers=[1])

ALL_SUITES_ALIASES = {
    'town1': [
        'FullTown01-v1', 'FullTown01-v2', 'FullTown01-v3', 'FullTown01-v4', 'StraightTown01-v1', 'StraightTown01-v2',
        'TurnTown01-v1', 'TurnTown01-v2'
    ],
    'town2': [
        'FullTown02-v1', 'FullTown02-v2', 'FullTown02-v3', 'FullTown02-v4', 'StraightTown02-v1', 'StraightTown02-v2',
        'TurnTown02-v1', 'TurnTown02-v2'
    ],
    'town4': [
        'FullTown04-v1', 'FullTown04-v2', 'FullTown04-v3', 'FullTown04-v4', 'StraightTown04-v1', 'StraightTown04-v2',
        'ChangeLaneTown04-v1', 'ChangeLaneTown04-v2'
    ],
    'train': ['FullTown01-v1', 'StraightTown01-v1', 'TurnTown01-v1'],
    'train_multilane': ['FullTown04-v1', 'StraightTown04-v1', 'ChangeLaneTown04-v1'],
    'train_ft': ['FullTown01-v1', 'TurnTown01-v1'],
    'train_veh': ['FullTown01-v1', 'FullTown01-v3'],
    'validate': ['FullTown01-v5', 'FullTown02-v5'],
    'test': ['FullTown02-v2', 'StraightTown02-v1', 'TurnTown02-v1'],
    'test_veh': ['FullTown02-v2', 'FullTown02-v4'],
    'town1p': [
        'FullTown01-v5',
        'FullTown01-v6',
        'StraightTown01-v3',
        'TurnTown01-v3',
        'FullTown01-v5',
        'FullTown01-v6',
    ],
    'town2p': [
        'FullTown02-v5',
        'FullTown02-v6',
        'StraightTown02-v3',
        'TurnTown02-v3',
        'FullTown02-v5',
        'FullTown02-v6',
    ],
    'ntown1p': [
        'NoCrashTown01-v7',
        'NoCrashTown01-v8',
        'NoCrashTown01-v9',
    ],
    'ntown2p': [
        'NoCrashTown02-v7',
        'NoCrashTown02-v8',
        'NoCrashTown02-v9',
    ],
    'empty': [
        'NoCrashTown01-v1',
        'NoCrashTown01-v2',
        'NoCrashTown02-v1',
        'NoCrashTown02-v2',
    ],
    'regular': [
        'NoCrashTown01-v3',
        'NoCrashTown01-v4',
        'NoCrashTown02-v3',
        'NoCrashTown02-v4',
    ],
    'regular-np': [
        'NoCrashTown01-v3-np',
        'NoCrashTown01-v4-np',
        'NoCrashTown02-v3-np',
        'NoCrashTown02-v4-np',
    ],
    'dense': [
        'NoCrashTown01-v5',
        'NoCrashTown01-v6',
        'NoCrashTown02-v5',
        'NoCrashTown02-v6',
    ],
    'dense-np': [
        'NoCrashTown01-v5-np',
        'NoCrashTown01-v6-np',
        'NoCrashTown02-v5-np',
        'NoCrashTown02-v6-np',
    ]
}

ALL_SUITES_ALIASES['all'] = ALL_SUITES_ALIASES['town1'] + \
    ALL_SUITES_ALIASES['town2']
