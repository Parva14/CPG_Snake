'''
## lookup - Classes related to finding modules on the network

This module contains the one class which should be instanced, HebiLookup. Use
this as your entry point into the API.
'''

from __future__ import print_function
import time

import ctypes
from hebiapi.base import hebi, HebiPointer, returnArray, HebiAccessError
from hebiapi.group import HebiGroup


class HebiLookup(HebiPointer):
    '''Python wrapping of HebiLookup in HEBI API

    This object represents a HebiLookup instance, and is used to find HEBI
    modules on the network. This is the standard entry point into the module.
    This class lets you search and find modules on your network, and create
    HebiGroups containing modules. Only create one HebiLookup instance per
    program.
    '''
    def __init__(self, printData=True):
        super().__init__(hebi.hebiLookupCreate())
        time.sleep(0.01)
        self.lookupList = self._getLookupEntryList()
        if printData:
            self._printLookupTable()

    def _close(self, addr):
        hebi.hebiLookupRelease(addr)

    def _printLookupTable(self):
        hebi.hebiPrintLookupTable(self.getAddress())

    def _getLookupEntryList(self):
        return LookupEntryList(
            hebi.hebiLookupCreateLookupEntryList(self.getAddress())
        )

    def _getNamesAndFamilies(self):
        return self.lookupList.getNamesAndFamilies()

    def getNumModules(self):
        '''Gets the number of modules found by the lookup

        Returns:

            int: number of modules found'''
        return self.lookupList.getNumEntries()

    def getNamesAndFamilies(self):
        '''This function gets the names and families of modules found by the lookup

        Returns:

            (str, str) list: a list of 2-tuples which each contain the name and
                             family of a module found by the lookup. The length
                             of the list is the same as the value returned by
                             getNumModules().
        '''
        return returnArray(self.lookupList.getNamesAndFamilies())

    def getMacFromIndex(self, index):
        '''Gets the MAC address of a specific module.

        Args:

            index (int): The index of the module in the list returned by
                         getNamesAndFamilies(). Should be at least 0 and less
                         than getNumModules().

        Returns:

            macAddress (str): The MAC address of the module.
        '''
        return self.lookupList.getMac(index)

    def getFamilyFromIndex(self, index):
        '''Gets the family of a module.

        Args:

            index (int): The index of the module in the list returned by
                         getNamesAndFamilies(). Should be at least 0 and less
                         than getNumModules().

        Returns:

            string: The family of the module.
        '''
        return self.lookupList.getFamily(index)

    def getNameFromIndex(self, index):
        '''Gets the name of a module.

        Args:

            index (int): The index of the module in the list returned by
                         getNamesAndFamilies(). Should be at least 0 and less
                         than getNumModules().

        Returns:

            string: The name of the module.
        '''
        return self.lookupList.getName(index)

    def getGroupFromNames(self, names,
                          families='*',
                          timeout=1000,
                          commandLifetime=3000):
        '''Gets a HebiGroup representing a group of connected modules from a list.

        If families is '`*`', then '`*`' is used as the family for all
        modules. If '`*`' is the family of a module, then any module with the
        name is used.

        Args:

            names    (str list): A list of the names of all the modules
            families (str list): A list of the families of all the modules.
                                 '*' is a wildcard character.
            timeout       (int): the time in milliseconds that is given to
                                 create the group.
            commandLifetime   (int): the time in milliseconds commnads sent to
                                     the group will persist on the group. If
                                     commandLifetime is 0, commands will
                                     persist on the group indefinitely.
        '''
        if type(names) != list:
            raise HebiAccessError('Expected a list, got a {}'.format(type(names)))
        if families == '*':
            families = ['*' for name in names]
        if '*' in families:
            modules = self._getNamesAndFamilies()
        matchedModules = []
        for name, family in zip(names, families):
            if family == '*':
                for module in filter(lambda m: m[0] == name, modules):
                    matchedModules.append(module)
            else:
                matchedModules.append((name, family))
        namesBuffer = (ctypes.c_char_p * len(matchedModules))()
        familiesBuffer = (ctypes.c_char_p * len(matchedModules))()
        for i, module in enumerate(matchedModules):
            namesBuffer[i] = module[0].encode()
            familiesBuffer[i] = module[1].encode()
        group = HebiGroup(names[0], families[0],
                          addr=hebi.hebiCreateGroupFromNames(
                              self.getAddress(),
                              namesBuffer,
                              len(matchedModules),
                              familiesBuffer,
                              len(matchedModules),
                              timeout
                          ))
        group.setCommandLifetime(commandLifetime)
        return group

    def getConnectedGroupFromName(self, name,
                                  family='*',
                                  timeout=1000,
                                  commandLifetime=3000):
        '''Gets a HebiGroup representing a group of connected modules from a
        root module.

        Args:

            name        (str): the name of the root module
            family      (str): the family of the root module. '*' represents a
                               wildcard. If using '*' and multiple modules have
                               the same name, one is chosen arbitrarily to be
                               root.
            timeout     (int): the time in milliseconds that is given to create
                               the group.
            commandLifetime (int): the time in milliseconds commands sent to
                                   the group will persist on the group. If
                                   commandLifetime is 0, commands will persist
                                   on the group indefinitely.

        Returns:

            HebiGroup (hebiapi.group.HebiGroup): A HebiGroup instance which
                                                 represents the group of
                                                 modules connected to the root
                                                 module.
        '''
        if family == '*':
            modules = self._getNamesAndFamilies()
            matchedModules = list(filter(lambda m: m[0] == name, modules))
            if not matchedModules:
                print('No module with name {} found!'.format(name))
                return None
            name, family = matchedModules[0]
        group = HebiGroup(name, family,
                          addr=hebi.hebiCreateConnectedGroupFromName(
                              self.getAddress(),
                              name.encode(),
                              family.encode(),
                              timeout
                          ))
        group.setCommandLifetime(commandLifetime)
        return group


class LookupEntryList(HebiPointer):
    ''' Wrapper for extracting information from a HebiLookup. Not normally needed!

        Internally used class which aids in extracting information from
        HebiLookup. This should not be directly used, but instead used through
        an instance of a HebiLookup object.
        '''

    def _close(self, addr):
        hebi.hebiLookupEntryListRelease(addr)

    def getNumEntries(self):
        '''See HebiLookup.getNumModules()'''
        return hebi.hebiLookupEntryListGetNumberOfEntries(self.getAddress())

    def getName(self, index):
        '''See HebiLookup.getName()'''
        buf = ctypes.create_string_buffer(1)
        bufLen = hebi.hebiLookupEntryListGetName(
            self.getAddress(),
            index,
            buf,
            0
        )
        buf = ctypes.create_string_buffer(bufLen)
        if hebi.hebiLookupEntryListGetName(
                self.getAddress(),
                index,
                buf,
                bufLen
        ) != 0:
            print(
                'Something went wrong getting name for module {}'.format(index)
            )
            return None
        return buf.value.decode()

    def getFamily(self, index):
        '''See HebiLookup.getFamily()'''
        buf = ctypes.create_string_buffer(1)
        bufLen = hebi.hebiLookupEntryListGetFamily(
            self.getAddress(), index, buf, 0
        )
        buf = ctypes.create_string_buffer(bufLen)
        if hebi.hebiLookupEntryListGetFamily(
                self.getAddress(), index, buf, bufLen
        ) != 0:
            print(
                'Something went wrong getting family {}'.format(index)
            )
            return None
        return buf.value.decode()

    def getMac(self, index):  # Not working under windows
        '''See HebiLookup.getMac(). Not functional under windows.'''
        return ':'.join(
            [hex(byte).replace('0x', '') for byte in
             list(
                 hebi.hebiLookupEntryListGetMacAddress(self.getAddress(),
                                                       index).bytes_
             )]
        )

    def getNamesAndFamilies(self):
        '''See HebiLookup.getNumModules()'''
        modules = []
        for module in range(self.getNumEntries()):
            modules.append((self.getName(module), self.getFamily(module)))
        return modules
