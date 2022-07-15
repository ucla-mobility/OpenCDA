import os
import re
import numpy as np
import pandas as pd
from typing import Union, List
from pathlib import Path
from terminaltables import DoubleTable

from opencda.core.ml_libs.rl.scenario_data.benchmark_suite import ALL_SUITES, ALL_SUITES_ALIASES


def get_suites_list(suite_name) -> List:
    all_suites_name = list(ALL_SUITES.keys()) + list(ALL_SUITES_ALIASES.keys())
    suite_list = []
    if isinstance(suite_name, list):
        for suite in suite_name:
            if suite.lower() in all_suites_name:
                suite_list += ALL_SUITES_ALIASES[suite]
            else:
                suite_list += [suite]
    else:
        if suite_name.lower() in all_suites_name:
            suite_list = ALL_SUITES_ALIASES[suite_name]
        else:
            suite_list = [suite_name]
    return suite_list


def read_pose_txt(benchmark_dir, poses_txt) -> Union[int, int]:
    pairs_file = Path(benchmark_dir) / poses_txt
    pose_pairs = pairs_file.read_text().strip().split('\n')
    pose_pairs = [(int(x[0]), int(x[1])) for x in map(lambda y: y.split(), pose_pairs)]

    return pose_pairs


def gather_results(result_dir) -> None:
    performance = dict()
    result_dir = Path(result_dir)
    for summary_path in result_dir.glob('*.csv'):
        name = summary_path.name
        match = re.search('^(?P<suite_name>.*Town.*-v[0-9]+.*)_seed(?P<seed>[0-9]+)', name)
        suite_name = match.group('suite_name')
        seed = match.group('seed')

        summary = pd.read_csv(summary_path)

        if suite_name not in performance:
            performance[suite_name] = dict()

        performance[suite_name][seed] = (summary['success'].sum(), len(summary))

    table_data = []
    for suite_name, seeds in performance.items():
        successes, totals = np.array(list(zip(*seeds.values())))
        rates = successes / totals * 100

        if len(seeds) > 1:
            table_data.append(
                [
                    suite_name,
                    "%.1f ± %.1f" % (np.mean(rates), np.std(rates, ddof=1)),
                    "%d/%d" % (sum(successes), sum(totals)), ','.join(sorted(seeds.keys()))
                ]
            )
        else:
            table_data.append(
                [
                    suite_name,
                    "%d" % np.mean(rates),
                    "%d/%d" % (sum(successes), sum(totals)), ','.join(sorted(seeds.keys()))
                ]
            )

    table_data = sorted(table_data, key=lambda row: row[0])
    table_data = [('Suite Name', 'Success Rate', 'Total', 'Seeds')] + table_data
    table = DoubleTable(table_data, "Performance of %s" % result_dir.name)
    return table.table


def get_benchmark_dir():
    main_dir = os.getcwd()
    current_work_dir = os.path.dirname(__file__)
    benchmark_dir = os.path.join(main_dir, current_work_dir)
    return benchmark_dir
