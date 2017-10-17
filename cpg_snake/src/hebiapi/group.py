'''
group - Contains classes and objects for interacting with groups of modules.
============================================================================

HebiGroup.setAngles, HebiGroup.setTorques, HebiGroup.getAngles, and
HebiGroup.getTorques are the preferred way of controlling the modules.
'''
from __future__ import print_function

import ctypes

from hebiapi.base import hebi, HebiPointer, HebiAccessError, \
                         flatten, returnArray
from hebiapi.command import HebiGroupCommand
from hebiapi.feedback import HebiGroupFeedback, HebiGroupInfo
from hebiapi.constants import Commands, defaultGains, Info

__all__ = ['HebiGroup']


class _Gains(dict):
    def __init__(self, callback, *args, **kwargs):
        self.callback = callback
        dict.__init__(self, *args, **kwargs)

    def __setitem__(self, *args, **kwargs):
        key, value = args[:2]  # pylint: disable=unbalanced-tuple-unpacking
        if key in self.keys():
            args = list(args)
            args[1] = flatten(value)
            previous = _Gains(self.callback, self)
            dict.__setitem__(self, *args, **kwargs)
            self.callback(previous, args[1])
        else:
            raise HebiAccessError('Cannot add additional gains!')

    def _doNothing(*args, **kwargs):  # pylint: disable=no-method-argument
        pass

    __delitem__ = _doNothing
    clear = _doNothing
    pop = _doNothing
    popitem = _doNothing
    setdefault = _doNothing
    update = _doNothing


class HebiGroup(HebiPointer):
    '''Object representing a group of modules.

    You should not create a HebiGroup directly. Instead use a HebiLookup to
    create a group.

    Attributes:

        groupInfo        (HebiGroupInfo): Internal HebiGroupInfo used for
                                          getting info from group.
        groupFeedback(HebiGroupFeedback): Internal HebiGroupFeedback used for
                                          getting feedback from group.
        groupCommand  (HebiGroupCommand): Internal HebiGroupCommand used for
                                          sending commands to the group.
        groupGains   (string:float dict): Dictionary representing the gains of
                                          the module
        rootName                   (str): Name of the root module.
        rootFamily                 (str): Family of the root module.
        '''
    rootName = ''
    rootFamily = ''

    def __init__(self, rootName, rootFamily, addr=0):
        self.rootName = rootName
        self.rootFamily = rootFamily
        HebiPointer.__init__(self, addr)
        self.groupCommand = HebiGroupCommand(self.getNumModules())
        self.groupFeedback = HebiGroupFeedback(self.getNumModules())
        self.groupInfo = HebiGroupInfo(self.getNumModules())
        self.updateGains()
        self._gainsSet = False

    def _close(self, addr):
        #    hebi.hebiReleaseGroup(self.getAddress())
        pass

    def _getFeedbacks(self, **kwargs):
        self.groupFeedback.requestFeedback(self, **kwargs)
        # pylint: disable=protected-access
        return self.groupFeedback._getFeedbackList()

    def _getInfos(self, **kwargs):
        self.groupInfo.requestInfo(self, **kwargs)
        # pylint: disable=protected-access
        return self.groupInfo._getInfoList()

    def _updateGains(self, previous, values):
        if len(values) != self.getNumModules():
            # pylint: disable=attribute-defined-outside-init
            self.groupGains = previous
            raise HebiAccessError(
                'Length of set gains values must be the number of modules in \
                the group!'
            )
        self._setGains()

    def _setGains(self):
        for gain in self.groupGains:
            if all(modGain is not None for modGain in self.groupGains[gain]):
                if gain == 'ControlStrategy':
                    self.groupCommand.setEnums(
                        vars(Commands)['CommandEnum'+gain],
                        self.groupGains[gain]
                    )
                else:
                    self.groupCommand.setFields(
                        vars(Commands)['CommandFloat'+gain],
                        self.groupGains[gain]
                    )
        self._gainsSet = True

    def setGains(self, gainsDict):
        '''Sets all the gains to a dictionary of values.

        Args:

            gainsDict (str:float dict): the dictionary which represents the
                                        gains of the modules.
        '''
        if set(gainsDict.keys()) == set(self.groupGains.keys()):
            self.groupGains = _Gains(self._updateGains, gainsDict)
            self._setGains()
        else:
            raise HebiAccessError('Gains dict given has invalid fields.')

    def updateGains(self, **kwargs):
        '''Updates the internal gains values to match that of the modules

        Args:

            timeout (int): time, in milliseconds, for the C API to set gains
        '''
        moduleInfos = self._getInfos(**kwargs)
        newGains = {gainName: [] for gainName in defaultGains}
        for info in moduleInfos:
            for gain in defaultGains:
                if gain == 'ControlStrategy':
                    value = info.getEnum(vars(Info)['InfoEnum'+gain])
                else:
                    value = info.getField(vars(Info)['InfoFloat'+gain])
                newGains[gain].append(value)
        self.groupGains = _Gains(self._updateGains, newGains)

    def getNumModules(self):
        '''Gets the number of modules in the group.

        Returns:

            numModules (int): the number of modules in the group'''
        return hebi.hebiGroupGetNumberOfModules(self.getAddress())

    def sendCommand(self, command=None, release=False):
        '''Sends a command to the group.

        Args:

            command (HebiGroupCommand): the command to send. If this is None,
                                        then the internally stored command is
                                        used.
            release             (bool): True releases and resets the command,
                                        object, while False does not'''
        if not command:
            command = self.groupCommand
        command.sendCommand(self, release=(release or self._gainsSet))
        self._gainsSet = False

    def setAngles(self, angles, send=True):
        '''Sets the angles of the modules in the group.

        The length of angles must be the length of the modules in the group. If
        an angle is None, the corresponding module is not sent an angle
        command.

        Args:

            angles (float list): The target angles
            send         (bool): True sends the command immediately, and False
                                 only updates the internal HebiGroupCommand
                                 object.
        '''
        if len(angles) != self.getNumModules():
            raise HebiAccessError(
                'Length of angles given was {} but group contains {} modules.'
                .format(len(angles), self.getNumModules())
            )
        angles = flatten(angles)
        commands = self.groupCommand.getCommandList()
        for i, angle in enumerate(angles):
            if angle is not None:
                commands[i].setAngle(angle)
        if send:
            self.sendCommand()

    def setTorques(self, torques, send=True):
        '''Sets the output torques of the modules in the group.

        The length of torques must be the length of the modules in the group.
        If a torque is None, the corresponding module is not sent an torque
        command.

        Args:

            torques (float list): The target torques
            send          (bool): True sends the command immediately, and
                                  False only updates the internal
                                  HebiGroupCommand object.
        '''
        if len(torques) != self.getNumModules():
            raise HebiAccessError(
                'Length of torques given was {} but group contains {} modules.'
                .format(len(torques), self.getNumModules())
            )
        torques = flatten(torques)
        commands = self.groupCommand.getCommandList()
        for i, torque in enumerate(torques):
            if torque is not None:
                commands[i].setField(Commands.CommandFloatTorque, torque)
        if send:
            self.sendCommand()

    def getFeedbacks(self, wait=True, **kwargs):
        '''Gets the feedbacks for all the modules in the group.

        getFeedbacks returns a list of feedback objects, which can be queried
        to get feedback from the module they represent.

        Args:

            timeout (int): Time, in milliseconds, to wait before timing out
                           when waiting for a response from the group.

        Returns:

            feedbacks (HebiFeedback list): a list containing the feedbacks
                                           corresponding to each module in the
                                           group.
            wait                   (bool): if True, then blocks code until
                                           feedback is received.
        '''
        return returnArray(self._getFeedbacks(wait=wait, **kwargs))

    def getFeedback(self, wait=True, **kwargs):
        '''Gets the feedback for all the modules in the group.

        This function is more similar to the feedback given in the matlab API,
        in that the feedback object returned gives lists of data points,
        rather than the list of feedback objects as in getFeedbacks.

        Args:

            timeout (int): Time, in milliseconds, to wait before timing out
                           when waiting for a response from the group.

        Returns:

            feedbacks (HebiFeedback list): a list containing the feedbacks
                                           corresponding to each module in the
                                           group.
            wait                   (bool): if True, then blocks code until
                                           feedback is received.
        '''
        self.groupFeedback.requestFeedback(self, wait=wait, **kwargs)
        return self.groupFeedback

    def getInfo(self, **kwargs):
        '''Gets the feedbacks for all the modules in the group.

        This function is more similar to the info given in the matlab API, in
        that the info object returned gives lists of data points, rather than
        the list of info objects as in getInfos.

        Args:

            timeout (int): Time, in milliseconds, to wait before timing out
                           when waiting for a response from the group.

        Returns:

            infos (HebiGroupInfo): Info object for the group.
        '''
        self.groupInfo.requestInfo(self, **kwargs)
        return self.groupInfo

    def getInfos(self, **kwargs):
        '''Gets the feedbacks for all the modules in the group.

        Args:

            timeout (int): Time, in milliseconds, to wait before timing out
                           when waiting for a response from the group.

        Returns:

            infos (HebiInfo list): a list containing the infos corresponding
                                   to each module in the group.
        '''
        return returnArray(self._getInfos(**kwargs))

    def _getAngles(self, **kwargs):
        feedbackList = self._getFeedbacks(**kwargs)
        angleTuples = [moduleFbk.getPosition() for moduleFbk in feedbackList]
        if any([tuple is None for tuple in angleTuples]):
            return None
        return [moduleFbk.getPosition()[1] for moduleFbk in feedbackList]

    def getAngles(self, **kwargs):
        '''Gets the current angles of all the modules in the group.

        Args:

            timeout (int): Time, in milliseconds to wait before timing out when
                           waiting for a response from the group.

        Returns:

            angles (float list): A list containing the current angles of the
                                 modules in the group.
        '''
        return returnArray(self._getAngles(**kwargs))

    def _getTorques(self, **kwargs):
        feedbackList = self._getFeedbacks(**kwargs)

        return [moduleFbk.getTorque() for moduleFbk in feedbackList]

    def getTorques(self, **kwargs):
        '''Gets the current torques of all the modules in the group.

        Args:

            timeout (int): Time, in milliseconds, to wait before timing out
                           when waiting for a response from the group.

        Returns:

            torques (float list): A list containing the current angles of
                                  the modules in the group.
        '''
        return returnArray(self._getTorques(**kwargs))

    def getModuleNamesAndFamilies(self, timeout=1000, **kwargs):
        '''Gets the module names and families, in a similar fashion to
        HebiLookup.getNamesAndFamilies()

        Args:

            timeout (int): Time, in milliseconds, to wait before timing out
                           when waiting for a response from the group.

        Returns:

            (str, str) list: a list of 2-tuples which each contain the name and
                             family of a module found by the lookup. The length
                             of the list is the same as the value returned by
                             getNumModules().
        '''

        info = self._getInfos(timeout=timeout, **kwargs)
        modules = []
        if len(info) == self.getNumModules():
            for module in range(self.getNumModules()):
                modules.append(
                    (info[module].getName(), info[module].getFamily())
                )
        return returnArray(modules)

    def setFeedbackFrequency(self, hz):
        '''Sets the feedback frequency of the group (the frequency of the
        underlying HEBI API)

        Args:

            hz (float): value, in hertz, to set the frequency'''
        hebi.hebiGroupSetFeedbackFrequencyHz(self.getAddress(),
                                             ctypes.c_float(hz))

    def getFeedbackFrequency(self):
        '''Gets the feedback frequency of the group (the frequency of the
        underlying HEBI API

        Returns:

            hz (float): value, in hertz of the loop frequency.
        '''
        return hebi.hebiGroupGetFeedbackFrequencyHz(self.getAddress())

    def getCommandLifetime(self):
        '''Gets the command lifetime of the group (the length of time which
        commands persist on the group).

        Returns:

            commandLifetime (int): the command lifetime, in milliseconds. If
                                   0, then commands remain active until the
                                   next command is received.
        '''
        return hebi.hebiGroupGetCommandLifetime(self.getAddress())

    def setCommandLifetime(self, commandLifetime):
        '''Sets the command lifetime of the group (how long commands
        persist on the group).

        Args:

            cmdLifetime (int): lifetime, in milliseconds, of
                               commands. If 0, commands persist
                               indefinitely.
        '''
        hebi.hebiGroupSetCommandLifetime(self.getAddress(), commandLifetime)
