import os
from distutils.core import setup

CARLA_VERSION = '0.9.11'
if 'CARLA_VERSION' in os.environ:
      CARLA_VERSION = os.environ['CARLA_VERSION']


setup(name='carla',
      version=CARLA_VERSION,
      py_modules=['carla'],
      )
