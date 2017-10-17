'''
## command - Classes and objects for sending commands to modules

Be wary when using anything from this module, as most of it is for internal
use. If you think you need something from here, there is probably a safer way
of doing what you want using a different module.
'''

from __future__ import print_function

import ctypes
from hebiapi.base import hebi, HebiPointer, flatten
from hebiapi.constants import Commands


class HebiCommand(HebiPointer):
    '''Object used to encapsulate HebiCommands. This should never be created
    directly!
    '''
    def setAngle(self, angle):
        '''Sets the angle of the command pointer represented by this
        HebiCommand object.

        Additionally clips the angle between -pi/2 and pi/2

        Args:

            angle (float): the desired angle
        '''
        decPart = ctypes.c_float(angle)
        hebi.hebiCommandSetHighResAngle(self.getAddress(),
                                        Commands.CommandHighResAnglePosition,
                                        ctypes.c_int(),
                                        decPart)

    def setField(self, field, value):
        '''Sets an arbitrary field of the command pointer.

        Should not normally be used directly, proceed with caution. Field is
        defined in hebiapi.constants.

        Args:

            field (int)  : the field to be set
            value (float): the value to be set
        '''
        hebi.hebiCommandSetFloat(self.getAddress(),
                                 field,
                                 ctypes.c_float(value))

    def setEnum(self, field, value):
        '''Sets an arbitrary field of the command pointer.

        Should not normally be used directly, proceed with caution. Field is
        defined in hebiapi.constants.

        Args:

            field (int): the field to be set
            value (int): the value to be set
        '''
        hebi.hebiCommandSetEnum(self.getAddress(), field, ctypes.c_int(value))

    def getField(self, field):
        '''Gets the value of an arbitrary field of the command pointer.

        Should not normally be used directly, proceed with caution. Field is
        defined in hebiapi.constants.

        Args:

            field (int): the field to get the value from

        Returns:

            value (float): the value of the field
        '''
        if hebi.hebiCommandHasFloat(self.getAddress(), field):
            return hebi.hebiCommandGetFloat(self.getAddress(), field)


class HebiGroupCommand(HebiPointer):
    '''Object used to encapsulate a HebiGroupCommand (command for entire
    group).

        Should not normally be used directly, proceed with caution.

    Attributes:

        numModules (int): the number of modules represented by this
                          HebiGroupCommand

    Args:

        create (bool): whether to initialze the command automatically
        addr    (int): the initialzation address. Required if create == False,
                       ignored if create == True.
    '''
    def __init__(self, numModules, create=True, addr=0):
        if create:
            addr = hebi.hebiGroupCommandCreate(numModules)
        self.numModules = numModules
        HebiPointer.__init__(self, addr)

    def _close(self, addr):
        hebi.hebiGroupCommandRelease(addr)

    def sendCommand(self, group, release=False):
        '''Sends the group command to the group

        Args:

            group (HebiGroup): the target group
            release    (bool): whether to release command after use (i.e.
                               reset command fields)
        '''
        hebi.hebiGroupSendCommand(group.getAddress(), self.getAddress())
        if release:
            hebi.hebiGroupCommandRelease(self.getAddress())
            HebiPointer.__init__(self,
                                 hebi.hebiGroupCommandCreate(self.numModules))

    def getCommandList(self):
        '''Gets the commands for each module in the group

        Should not normally be used directly, proceed with caution.

        Returns:

            commandList (HebiCommand list): a list of HebiCommands which are
                                            tied to each module in the group.
        '''
        cmdList = []
        for i in range(self.numModules):
            cmd = HebiCommand(
                hebi.hebiGroupCommandGetModuleCommand(self.getAddress(), i)
            )
            cmdList.append(cmd)
        return cmdList

    def setFields(self, field, values):
        '''Sets the field of all the modules to the floats in the values
        array.

        Should not normally be used directly, proceed with caution.

        Args:

            field         (int): the field to be set
            value (float array): the values to set
        '''
        values = flatten(values)
        for command, value in zip(self.getCommandList(), values):
            command.setField(field, value)

    def setEnums(self, field, values):
        '''Sets the field of all the modules to the ints in the values array.

        Should not normally be used directly, proceed with caution.

        Args:

            field         (int): the field to be set
            value   (int array): the values to set'''
        values = flatten(values)
        for command, value in zip(self.getCommandList(), values):
            command.setEnum(field, value)
