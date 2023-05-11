from distutils.core import setup
from Cython.Build import cythonize
import numpy
setup(
    name='box overlaps',
    ext_modules=cythonize('opencood/utils/box_overlaps.pyx'),
    include_dirs=[numpy.get_include()]
)