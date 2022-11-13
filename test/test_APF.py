# -*- coding: utf-8 -*-
"""
Unit test for All Predecessor Following.
"""
# Author: XH <@ucla.edu>
# License: MIT

import os
import sys
import unittest

# temporary solution for relative imports in case opencda is not installed
# if opencda is installed, no need to use the following line
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '.')))

import mocked_platoon as mplatoon
from opencda.core.application.platooning.platoon_APF import AllPredesessorFollowing


class testAPF(unittest.TestCase):
    def setUp(self):
        # init speed for all (8 m/s = 18 mph)
        speed = mplatoon.Vector3D(x=0, y=8, z=0)
        # mark platoon speed
        self.platoon_speed = speed
        # init a list of vehicles
        vm0 = mplatoon.VehicleManager(mplatoon.Vehicle(mplatoon.Transform(x=12, y=4, z=12), speed))
        vm1 = mplatoon.VehicleManager(mplatoon.Vehicle(mplatoon.Transform(x=12, y=12, z=12), speed))
        vm2 = mplatoon.VehicleManager(mplatoon.Vehicle(mplatoon.Transform(x=12, y=20, z=12), speed))
        vm3 = mplatoon.VehicleManager(mplatoon.Vehicle(mplatoon.Transform(x=12, y=28, z=12), speed))
        vm4 = mplatoon.VehicleManager(mplatoon.Vehicle(mplatoon.Transform(x=12, y=36, z=12), speed))
        vm5 = mplatoon.VehicleManager(mplatoon.Vehicle(mplatoon.Transform(x=12, y=44, z=12), speed))

        # init vm list 
        '''
        Initialization description:
            Default scenario: |v0----v1----v2----v3----v4----v5|
            Host position:                             ^
            Default gap size: 8m
        '''
        vehicle_manager_list = [vm0, vm1, vm2, vm3, vm4, vm5]

        # identify host location (starts with 0, i.e., the leader)
        current_in_id = 4

        # init APF
        self.APF = AllPredesessorFollowing(vehicle_manager_list, current_in_id)

    def reset_locations(self):
        '''
        Reset platoon vehicle positions for next tests.
        '''
        # relocate vehicles
        self.APF.vehicle_manager_list[0].vehicle.set_location(
            mplatoon.Transform(x=12, y=4, z=12))
        self.APF.vehicle_manager_list[1].vehicle.set_location(
            mplatoon.Transform(x=12, y=12, z=12))
        self.APF.vehicle_manager_list[2].vehicle.set_location(
            mplatoon.Transform(x=12, y=20, z=12))
        self.APF.vehicle_manager_list[3].vehicle.set_location(
            mplatoon.Transform(x=12, y=28, z=12))
        self.APF.vehicle_manager_list[4].vehicle.set_location(
            mplatoon.Transform(x=12, y=36, z=12))
        self.APF.vehicle_manager_list[5].vehicle.set_location(
            mplatoon.Transform(x=12, y=44, z=12))


    def test_parameters(self):
        # construction variabes
        assert (hasattr(self.APF, 'vehicle_manager_list') and
                len(self.APF.vehicle_manager_list) == 6)
        assert (hasattr(self.APF, 'current_in_id') and
                self.APF.current_in_id == 4)
        assert (hasattr(self.APF, 'current_dynamic_leader_index') and
                self.APF.current_dynamic_leader_index == "")
        assert (hasattr(self.APF, 'previous_dynamic_leader_index') and
                self.APF.previous_dynamic_leader_index == "")

        # functional variables
        assert (hasattr(self.APF, 'dynamic_leader_index_buffer') and
                len(self.APF.dynamic_leader_index_buffer) == 0)
        assert (hasattr(self.APF, 'timeHeadways') and
                len(self.APF.timeHeadways) == 0)
        assert (hasattr(self.APF, 'partialTimeHeadways') and
                len(self.APF.partialTimeHeadways) == 0)

    def test_run_step(self):
        # ---------- [test 1]: initialize APF ----------
        current_dynamic_leader_index = self.APF.run_step()
        assert (current_dynamic_leader_index == 0)

        # ---------- [test 2]: too close too preceding vehicle ----------
        '''
        test description:
            Default scenario: |v0----v1----v2----v3----v4----v5|
            Host position:                             ^
            Testing scenario: |v0----v1----v2----v3-v4----v5|
            d-leader position:                   ^
            host position:                           ^
        '''
        # move vehicles
        changed_dist = 7.0
        original_position_4 = self.APF.vehicle_manager_list[4].vehicle.transform.location
        original_position_5 = self.APF.vehicle_manager_list[5].vehicle.transform.location
        # relocate vehicles
        self.APF.vehicle_manager_list[4].vehicle.set_location(
            mplatoon.Transform(original_position_4.x,
                               original_position_4.y - changed_dist,
                               original_position_4.z))
        self.APF.vehicle_manager_list[5].vehicle.set_location(
            mplatoon.Transform(original_position_5.x,
                               original_position_5.y - changed_dist,
                               original_position_5.z))
        # too close too predecesor, dynamic leader change to 3
        current_dynamic_leader_index = self.APF.run_step()
        assert (current_dynamic_leader_index, 3)

        # ---------- [test 3a]: increase gap between v1 and v2 ----------
        '''
        test description:
            Default scenario: |v0----v1----v2----v3----v4----v5|
            Host position:                             ^
            Testing scenario: |v0----v1---------------v2----v3----v4----v5|
            d-leader position:                        ^
            host position:                                        ^
        '''
        # reset all vehicle locations. and revert to following leader
        self.reset_locations()
        current_dynamic_leader_index = self.APF.run_step()
        assert (current_dynamic_leader_index == 0)
        # move vehicles
        increased_distance = self.platoon_speed.y * self.APF.maxAllowHeadway
        original_position_0 = self.APF.vehicle_manager_list[0].vehicle.transform.location
        original_position_1 = self.APF.vehicle_manager_list[1].vehicle.transform.location
        # relocate vehicles
        self.APF.vehicle_manager_list[0].vehicle.set_location(
            mplatoon.Transform(original_position_0.x,
                               original_position_0.y - increased_distance,
                               original_position_0.z))
        self.APF.vehicle_manager_list[1].vehicle.set_location(
            mplatoon.Transform(original_position_1.x,
                               original_position_1.y - increased_distance,
                               original_position_1.z))

        current_dynamic_leader_index = self.APF.run_step()
        assert (current_dynamic_leader_index == 2)

        # ---------- [test 3b]: decrease gap between v1 and v2 ----------
        '''
        test description:
            Default scenario: |v0----v1----v2----v3----v4----v5|
            Host position:                             ^
            Testing scenario: |v0----v1-v2----v3----v4----v5|
            d-leader position:        ^
            host position:                          ^
        '''
        # reset all vehicle locations. and revert to following leader
        self.reset_locations()
        current_dynamic_leader_index = self.APF.run_step()
        assert (current_dynamic_leader_index == 0)
        # move vehicles
        original_position_0 = self.APF.vehicle_manager_list[0].vehicle.transform.location
        original_position_1 = self.APF.vehicle_manager_list[1].vehicle.transform.location
        # relocate vehicles
        self.APF.vehicle_manager_list[0].vehicle.set_location(
            mplatoon.Transform(original_position_0.x,
                               original_position_0.y + 6.5,
                               original_position_0.z))
        self.APF.vehicle_manager_list[1].vehicle.set_location(
            mplatoon.Transform(original_position_1.x,
                               original_position_1.y + 6.5,
                               original_position_1.z))

        current_dynamic_leader_index = self.APF.run_step()
        assert (current_dynamic_leader_index == 1)

        # ---------- [test 4]: previous d-leader is not 0, no Partial headway violation, d-leader stable  ----------
        '''
        test description:
            APF case 4, when preceding gap and following gap are stable, host search for
            downstream violators (out of last step's "partial headway") to follow.
            Previous scenario: |v0----v1----v2----v3----v4----v5|
            d-leader position:              ^
            host position:                              ^
            Partial headway:   |---------h1----h2----h3----h4---|
            Testing scenario:  |v0-----------v1----v2----v3----v4----v5|
            d-leader and gap:         2.0s   ^  1.0s  1.0s  1.0s  1.0s
            host position:                                     ^
        '''
        # reset all vehicle locations. and revert to following leader
        self.reset_locations()
        self.APF.previous_dynamic_leader_index = 2
        # move vehicles
        original_position_0 = self.APF.vehicle_manager_list[0].vehicle.transform.location
        # relocate vehicles
        self.APF.vehicle_manager_list[0].vehicle.set_location(
            mplatoon.Transform(original_position_0.x,
                               original_position_0.y - 8,
                               original_position_0.z))

        current_dynamic_leader_index = self.APF.run_step()
        assert (current_dynamic_leader_index == 1)

        # ----- [test 5a]: previous d-leader is not 0, no Partial headway violation, d-leader front unstable -----
        '''
        test description:
            APF case 5, when preceding gap and following gap are not stable,
            host remain to follow previous leader, even further violators exist.
            ---------------------------------------------------------------------
            Previous scenario: |v0----v1----v2----v3----v4----v5|
            d-leader position:              ^
            host position:                              ^
            Partial headway:   |---------h1----h2----h3----h4---|
            ---------------------------------------------------------------------
            Testing scenario:  |v0-----------v1-----v2----v3----v4----v5|
            d-leader and gap:        2.0s      1.75s ^ 1.0s
            host position:                                     ^
        '''
        # reset all vehicle locations. and revert to following leader
        self.reset_locations()
        self.APF.previous_dynamic_leader_index = 2
        # move vehicles
        original_position_0 = self.APF.vehicle_manager_list[0].vehicle.transform.location
        original_position_1 = self.APF.vehicle_manager_list[1].vehicle.transform.location

        # relocate vehicles
        self.APF.vehicle_manager_list[0].vehicle.set_location(
            mplatoon.Transform(original_position_0.x,
                               original_position_0.y - 14,
                               original_position_0.z))
        self.APF.vehicle_manager_list[1].vehicle.set_location(
            mplatoon.Transform(original_position_1.x,
                               original_position_1.y - 6,
                               original_position_1.z))

        current_dynamic_leader_index = self.APF.run_step()
        assert (current_dynamic_leader_index == 2)

        # ----- [test 5b]: previous d-leader is not 0, no Partial headway violation, d-leader rear unstable -----
        '''
        test description:
            APF case 5, when preceding gap and following gap are not stable,
            host remain to follow previous leader, even further violators exist.
            ---------------------------------------------------------------------
            Previous scenario: |v0----v1----v2----v3----v4----v5|
            d-leader position:              ^
            host position:                              ^
            Partial headway:   |---------h1----h2----h3----h4---|
            ---------------------------------------------------------------------
            Testing scenario:  |v0-----------v1----v2 -- v3----v4----v5|
            d-leader and gap:        2.0s      1.0s ^ 0.65s
            host position:                                      ^
        '''
        # reset all vehicle locations. and revert to following leader
        self.reset_locations()
        self.APF.previous_dynamic_leader_index = 2
        # move vehicles
        original_position_0 = self.APF.vehicle_manager_list[0].vehicle.transform.location
        original_position_3 = self.APF.vehicle_manager_list[3].vehicle.transform.location
        original_position_4 = self.APF.vehicle_manager_list[4].vehicle.transform.location
        original_position_5 = self.APF.vehicle_manager_list[5].vehicle.transform.location
        # relocate vehicles
        self.APF.vehicle_manager_list[0].vehicle.set_location(
            mplatoon.Transform(original_position_0.x,
                               original_position_0.y - 8,
                               original_position_0.z))
        self.APF.vehicle_manager_list[3].vehicle.set_location(
            mplatoon.Transform(original_position_3.x,
                               original_position_3.y - 2.8,
                               original_position_3.z))
        self.APF.vehicle_manager_list[4].vehicle.set_location(
            mplatoon.Transform(original_position_4.x,
                               original_position_4.y - 2.8,
                               original_position_4.z))
        self.APF.vehicle_manager_list[5].vehicle.set_location(
            mplatoon.Transform(original_position_5.x,
                               original_position_5.y - 2.8,
                               original_position_5.z))

        current_dynamic_leader_index = self.APF.run_step()
        assert (current_dynamic_leader_index == 2)

        # ----- [test 6]: previous d-leader is not 0, Partial headway min violation -----
        # reset all vehicle locations. and revert to following leader
        self.reset_locations()
        self.APF.previous_dynamic_leader_index = 2
        # move vehicles
        original_position_0 = self.APF.vehicle_manager_list[0].vehicle.transform.location
        original_position_1 = self.APF.vehicle_manager_list[1].vehicle.transform.location
        self.APF.vehicle_manager_list[0].vehicle.set_location(
            mplatoon.Transform(original_position_0.x,
                               original_position_0.y + 4,
                               original_position_0.z))
        self.APF.vehicle_manager_list[1].vehicle.set_location(
            mplatoon.Transform(original_position_1.x,
                               original_position_1.y + 4,
                               original_position_1.z))

        current_dynamic_leader_index = self.APF.run_step()
        assert (current_dynamic_leader_index == 1)

        # ----- [test 7]: previous d-leader is not 0, Partial headway max violation -----
        # reset all vehicle locations. and revert to following leader
        self.reset_locations()
        self.APF.previous_dynamic_leader_index = 2
        # move vehicles
        original_position_0 = self.APF.vehicle_manager_list[0].vehicle.transform.location
        original_position_1 = self.APF.vehicle_manager_list[1].vehicle.transform.location
        self.APF.vehicle_manager_list[0].vehicle.set_location(
            mplatoon.Transform(original_position_0.x,
                               original_position_0.y - 8,
                               original_position_0.z))
        self.APF.vehicle_manager_list[1].vehicle.set_location(
            mplatoon.Transform(original_position_1.x,
                               original_position_1.y - 8,
                               original_position_1.z))

        current_dynamic_leader_index = self.APF.run_step()
        assert (current_dynamic_leader_index == 2)

        # ----- [test 8]: previous d-leader is not 0, Partial headway max and min violation -----
        # reset previous d-leader
        self.APF.previous_dynamic_leader_index = 2
        # move vehicles
        # relocate vehicles
        self.APF.vehicle_manager_list[0].vehicle.set_location(
            mplatoon.Transform(x=12, y=4, z=12))
        self.APF.vehicle_manager_list[1].vehicle.set_location(
            mplatoon.Transform(x=12, y=12, z=12))
        self.APF.vehicle_manager_list[2].vehicle.set_location(
            mplatoon.Transform(x=12, y=28, z=12))
        self.APF.vehicle_manager_list[3].vehicle.set_location(
            mplatoon.Transform(x=12, y=36, z=12))
        self.APF.vehicle_manager_list[4].vehicle.set_location(
            mplatoon.Transform(x=12, y=40, z=12))
        self.APF.vehicle_manager_list[5].vehicle.set_location(
            mplatoon.Transform(x=12, y=48, z=12))

        current_dynamic_leader_index = self.APF.run_step()
        assert (current_dynamic_leader_index == 3)

        # ----- [test 9]: previous d-leader is not 0, Partial headway min and max violation -----
        # reset all vehicle locations. and revert to following leader
        self.reset_locations()
        self.APF.previous_dynamic_leader_index = 2
        # move vehicles
        original_position_0 = self.APF.vehicle_manager_list[0].vehicle.transform.location
        original_position_1 = self.APF.vehicle_manager_list[1].vehicle.transform.location
        original_position_2 = self.APF.vehicle_manager_list[2].vehicle.transform.location
        self.APF.vehicle_manager_list[0].vehicle.set_location(
            mplatoon.Transform(original_position_0.x,
                               original_position_0.y - 8 + 4,
                               original_position_0.z))
        self.APF.vehicle_manager_list[1].vehicle.set_location(
            mplatoon.Transform(original_position_1.x,
                               original_position_1.y - 8 + 4,
                               original_position_1.z))
        self.APF.vehicle_manager_list[2].vehicle.set_location(
            mplatoon.Transform(original_position_2.x,
                               original_position_2.y - 8,
                               original_position_2.z))

        current_dynamic_leader_index = self.APF.run_step()
        assert (current_dynamic_leader_index == 3)

if __name__ == '__main__':
    unittest.main()
