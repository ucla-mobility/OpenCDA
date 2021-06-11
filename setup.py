from os.path import dirname, realpath
from setuptools import setup, find_packages, Distribution
from opencda.version import __version__


def _read_requirements_file():
    """Return the elements in requirements.txt."""
    req_file_path = '%s/requirements.txt' % dirname(realpath(__file__))
    with open(req_file_path) as f:
        return [line.strip() for line in f]


setup(
    name='OpenCDA',
    version=__version__,
    packages=find_packages(),
    url='https://github.com/ucla-mobility/OpenCDA.git',
    license='MIT',
    author='Runsheng Xu, Xu Han',
    author_email='rxx3386@ucla.edu',
    description='A framework for fast developing cooperative driving automation and autonomous '
                'vehicle modules in multi-resolution simulation environment"',
    long_description=open("README.md").read(),
    install_requires=_read_requirements_file(),
)
