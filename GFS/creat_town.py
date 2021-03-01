# -*- coding: utf-8 -*-

"""Construct CARLA map from open street map
"""
# Author: xh
# License: MIT

import carla

# Read the .osm data
f = open("/home/xuhan/Carla-0.9.10/OpenStreetMap_local/map_v7.1_mod.osm", 'r') # Windows will need to encode the file in UTF-8. Read the note below. 
osm_data = f.read()
f.close()

# Define the desired settings. In this case, default values.
settings = carla.Osm2OdrSettings()
# Convert to .xodr
xodr_data = carla.Osm2Odr.convert(osm_data, settings)

# save opendrive file
f = open("/home/xuhan/Carla-0.9.10/OpenStreetMap_local/map_output/map_v7.1_mod.xodr", 'w')
f.write(xodr_data)
f.close()