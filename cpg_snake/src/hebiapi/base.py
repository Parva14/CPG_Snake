'''
## base - Contains utility functions and basic classes.
'''

from __future__ import print_function
import ctypes
import os
import platform
import warnings

import numpy
from hebiapi.constants import API_VERSION

__all__ = ["HebiMacAddress",
           "HebiVector3f",
           "HebiAccessError",
           "HebiPointer",
           "NP_MODE"]

__pdoc__ = {'NP_MODE': 'If set to true, then lists will be returned as 1 x n\
             numpy arrays. If False, then lists are returned as python lists.'}

NP_MODE = True
_ROOT = os.path.abspath(os.path.dirname(__file__))


def _getLib(path):
    return os.path.join(_ROOT, 'hebi', path)


def _importHebi():
    if platform.system() == 'Windows':
        warnings.warn('Windows is not fully supported!')
        import struct
        architecture = 8 * struct.calcsize("P")
        if architecture == 32:
            hebiDll = ctypes.cdll.LoadLibrary(
                _getLib(os.path.join("lib", "win_x86", "hebi.dll")))
        if architecture == 64:
            hebiDll = ctypes.cdll.LoadLibrary(
                _getLib(os.path.join("lib", "win_x64", "hebi.dll")))
    elif platform.system() == 'Darwin':
        hebiDll = ctypes.cdll.LoadLibrary(
                _getLib(
                    os.path.join('lib', 'macosx_x64', 'libhebi.{}.dylib')
                    .format(API_VERSION)))
    else:
        if platform.system() != 'Linux':
            print('System not recognized, attempting to use linux library')
        hebiDll = ctypes.cdll.LoadLibrary(
            _getLib(
                os.path.join('lib', 'linux_x86-64', 'libhebi.so.{}')
                .format(API_VERSION)))
    return hebiDll


def flatten(array):
    '''Takes in a multidimensional list or numpy array and flattens it.

    Args:

        array (numpy.array | list): the array to be flattened

    Returns:

        array (list): the flattened array (according to numpy.ndarray.flatten)
    '''
    return list(numpy.array(array).flatten())


def returnArray(array):
    '''Takes a regular python 1D list and converts it to the proper array
    format as defined by NP_MODE.

    Args:

        array (list): array to be converted

    Returns:

        array (numpy.array | list): numpy vector or list, depending on NP_MODE
    '''
    if array is None or any(item is None for item in array):
        array = []
    if NP_MODE:
        return numpy.array([list(array)])
    return list(array)


# change this 6 if the number of bytes in a mac address ever changes
MAC_ADDRESS = ctypes.c_ubyte * 6


class HebiMacAddress(ctypes.Structure):
    # pylint: disable=too-few-public-methods
    '''Python wrapper around Hebi MAC address struct.'''
    _fields_ = [('bytes_', MAC_ADDRESS)]

    def __init__(self, macAddress, *args, **kwargs):
        if type(macAddress) == str:
            bytes = [int(byte, 16) for byte in macAddress.split(':')]
            if (len(bytes) != 6 or
                    any([(byte < 0 or byte > 255) for byte in bytes])):

                raise HebiAccessError('Invalid MAC address given')
            macAddress = MAC_ADDRESS(*bytes)
        super().__init__(macAddress, *args, **kwargs)


class HebiVector3f(ctypes.Structure):  # pylint: disable=too-few-public-methods
    '''Python wrapper around Hebi Vector3f struct.'''
    _fields_ = [('x', ctypes.c_float),
                ('y', ctypes.c_float),
                ('z', ctypes.c_float)]

hebi = _importHebi()  # TODO: change all instances of hebi to HEBI
hebi.hebiFeedbackGetVector3f.restype = HebiVector3f
hebi.hebiLookupEntryListGetMacAddress.restype = HebiMacAddress
hebi.hebiInfoGetString.restype = ctypes.c_char_p
hebi.hebiFeedbackGetFloat.restype = ctypes.c_float
hebi.hebiInfoGetFloat.restype = ctypes.c_float
hebi.hebiGroupGetFeedbackFrequencyHz.restype = ctypes.c_float


class HebiAccessError(IOError):
    ''' Used to raise errors related to the HEBI C API, normally null pointer
    or access violations'''


class HebiPointer:  # pylint: disable=too-few-public-methods
    '''Basic class which is used to encapsulate a C pointer

    Attributes:

        addr (int): the value of the memory address pointed to by the C pointer
    '''
    def __init__(self, addr):
        '''Initializes a HebiPointer, setting the internal stored address to
        addr.
        '''
        self.addr = addr

    def __del__(self):  # pylint: disable=invalid-name
        '''Calls self._close(addr), so any cleanup can be done.

        If addr is 0, (NULL pointer), _close() is not called to prevent
        segfaults.
        '''
        if self.addr != 0:
            self._close(self.addr)

    def _close(self, addr):
        '''Called when object is destroyed'''
        pass

    def getAddress(self):
        '''Gets the address held within the object. Raises an error if the
        pointer is NULL.

        Returns:

            addr (int): the value of the contained pointer.'''
        if not self.addr:
            raise HebiAccessError('Action called on unitialized HebiPointer')
        return self.addr
