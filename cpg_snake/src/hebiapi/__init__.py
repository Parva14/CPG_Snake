'''
## hebiapi - Python bindings for the HEBI C API.

This module serves as a python wrapper around the HEBI Robotics C API for
controlling their SEA modules. Other than HebiLookup, no class in this module
should be instantiated. The other classes are returned from making function
calls to already instanced clases.

Examples can be found in the /examples/ directory of the git.

#### quickExample.py

    import hebiapi
    # finds a module, and sets its angle

    MODULE_NAME = 'SA076'

    hebiLookup = hebiapi.HebiLookup()

    group = hebiLookup.getGroupFromNames([MODULE_NAME])

    group.setAngles([0])

'''
from __future__ import print_function
from hebiapi.lookup import HebiLookup
import hebiapi.base

__all__ = ['HebiLookup', 'returnsNumpyArrays']


def returnsNumpyArrays(*args):
    '''Sets the type of array output by functions in the module

    Change the type of iterable returned by functions in the module. For
    example, if returnsNumpyArrays is True, then a call to group.getTorques()
    could return

        np.array([[0,0]])

    instead of

        [0,0]

    if called with no arguments, returns the current status instead.
    Args:

        bool (bool): When True, all array-like functions return numpy arrays.
                     If False, all array-like functions return python lists.
    '''
    if args:
        hebiapi.base.NP_MODE = args[0]
    return hebiapi.base.NP_MODE
