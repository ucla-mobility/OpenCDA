#!/usr/bin/env python

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
import time
import carla

try:
    sys.path.append(glob.glob('./PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

_HOST_ = '127.0.0.1'
_PORT_ = 2000
_SLEEP_TIME_ = 2

def main():

    client = carla.Client(_HOST_, _PORT_)
    client.set_timeout(2.0)
#        world = client.load_world("Town05")
    world = client.get_world()
    origin_settings = world.get_settings()

    try:
        
        # print(help(t))
        # print("(x,y,z) = ({},{},{})".format(t.location.x, t.location.y,t.location.z))
        
        # print_coord(world)
        while True:
            t = world.get_spectator().get_transform()
            coordinate_str = "(x,y,z) = ({},{},{})".format(int(t.location.x), int(t.location.y),int(t.location.z))
            rotation_str = "(p,y,r) = ({},{},{})".format(int(t.rotation.pitch), int(t.rotation.yaw), int(t.rotation.roll))
            print (coordinate_str, '\t', rotation_str)
            time.sleep(_SLEEP_TIME_)
    
    # set_transform(new_transform)

    # except KeyboardInterrupt:
        
    #     pass

    finally:
        
    #    for actor in world.get_actors():
    #        actor.destroy()

       world.apply_settings(origin_settings)

if __name__ == '__main__':
    main()



