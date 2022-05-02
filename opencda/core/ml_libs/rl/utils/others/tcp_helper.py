'''
Copyright 2021 OpenDILab. All Rights Reserved:
Description:carla benchmark data collector for DI-drive
'''
import multiprocessing as mp

from ding.utils.system_helper import find_free_port


class TCPManager(object):

    def __init__(self, cfg):
        self._cfg = cfg
        self._tcp_list = []
        self._tcp_state_dict = mp.Manager().dict()

        self._setup()

    def _setup(self):
        self._tcp_list = parse_carla_tcp(self._cfg)
        for i in range(len(self._tcp_list)):
            self._tcp_state_dict[i] = False

    def release_tcp(self, i):
        self._tcp_state_dict[i] = False

    def __getitem__(self, index):
        assert index < len(self)
        self._tcp_state_dict[index] = True
        return self._tcp_list[index]

    def __len__(self):
        return len(self._tcp_list)

    def get_available_env(self):
        for i in range(len(self)):
            if not self._tcp_state_dict[i]:
                return i
        return -1


def parse_carla_tcp(server_cfg):
    carla_tcp_list = []
    host = server_cfg.carla_host
    for port in server_cfg.carla_ports:
        carla_tcp_list.append((host, port))

    return carla_tcp_list


def find_traffic_manager_port():
    return find_free_port(None)
