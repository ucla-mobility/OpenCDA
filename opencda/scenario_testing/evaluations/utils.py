# -*- coding: utf-8 -*-
"""
Utility functions for evaluation.
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import sys


def lprint(logfile, *argv):
    """
    Save string to log file.

    Args:
        -logfile (File): The log file path.
        -*argv (string or number): the string that needs to be saved
        into the log file.

    """

    # argument check
    if len(argv) == 0:
        sys.exit('Err: wrong usage of func lprint(). Argv has to be provided.')

    arg_all = argv[0] if isinstance(argv[0], str) else str(argv[0])
    for arg in argv[1:]:
        arg_all = arg_all + (arg if isinstance(arg, str) else str(arg))

    with open(logfile, 'a') as out:
        out.write(arg_all + '\n')
