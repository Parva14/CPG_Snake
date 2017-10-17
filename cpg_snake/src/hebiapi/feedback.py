'''
## feedback - Classes and objects for getting feedback and data from modules

No classes in this module should be created manually, but they may be returned
by other hebiapi objects.
'''
from __future__ import print_function
import ctypes

from hebiapi.constants import Info
from hebiapi.base import hebi, returnArray, HebiPointer


class HebiInfo(HebiPointer):
    '''Object to aid in extracting module information. Should not be directly
    created.
    '''
    def getFamily(self):
        '''Gets the family of the current moduleInfo.

        Causes a segfault when the family of the represented module has a
        string length greater than 256.

        Returns:

            family (str): the family of the module'''
        if not hebi.hebiInfoHasString(self.getAddress(),
                                      Info.InfoStringFamily):
            return None

        buf = ctypes.create_string_buffer(256)
        # I had to default to 256 bytes for the buffer because otherwise the
        # hebi call (lines 21, 40) causes a segfault problem with C API, max
        # string size is 256
        # bufLen = hebi.hebiInfoGetString(self.getAddress(),
        #                                 Info.InfoStringFamily, buf, 1)
        # buf = ctypes.create_string_buffer(bufLen)
        bufLen = 256
        if hebi.hebiInfoGetString(self.getAddress(),
                                  Info.InfoStringFamily,
                                  buf,
                                  bufLen):
            print('Something went wrong getting family info for module')
            return None
        return buf.value.decode()

    def getName(self):
        '''Gets the name of the current moduleInfo.

        Causes a segfault when the name of the represented module has a string
        length greater than 256.

        Returns:

            name (str): the name of the module'''
        if not hebi.hebiInfoHasString(self.getAddress(), Info.InfoStringName):
            return None

        buf = ctypes.create_string_buffer(256)
        # bufLen = hebi.hebiInfoGetString(self.getAddress(),
        #                                 Info.InfoStringName, buf, 1)
        # buf = ctypes.pointer(ctypes.create_string_buffer(bufLen))
        bufLen = 256
        if hebi.hebiInfoGetString(self.getAddress(),
                                  Info.InfoStringName,
                                  buf,
                                  bufLen):
            print('Something went wrong getting name info for module')
            return None
        return buf.value.decode()

    def getField(self, field):
        '''Gets a field of the current moduleInfo.

        Should not normally be used directly.

        Args:

            field (int): the field desired

        Returns:

            value (float): the value of the field'''
        if not hebi.hebiInfoHasFloat(self.getAddress(), field):
            return None
        return hebi.hebiInfoGetFloat(self.getAddress(), field)

    def getEnum(self, enum):
        '''Gets an enum of the current moduleInfo.

        Should not normally be used directly.

        Args:

            enum (int): the enum desired

        Returns:

            value (int): the enum of the field'''
        if not hebi.hebiInfoHasEnum(self.getAddress(), enum):
            return None
        return hebi.hebiInfoGetEnum(self.getAddress(), enum)


class HebiFeedback(HebiPointer):
    '''Object to aid in extracting module feedback. Should not be directly
    created.
    '''

    def _getField(self, field):
        if not hebi.hebiFeedbackHasFloat(self.getAddress(), field):
            return None
        return hebi.hebiFeedbackGetFloat(self.getAddress(), field)

    def _getVector3fField(self, field):
        if not hebi.hebiFeedbackHasVector3f(self.getAddress(), field):
            return None
        data = hebi.hebiFeedbackGetVector3f(self.getAddress(), field)
        return (data.x, data.y, data.z)

    def getGyro(self):
        '''Gets the current state of the gyros in the module.

        Returns:

            gyros (float, float, float): (x,y,z) gyroscope data.'''
        return self._getVector3fField(Info.FeedbackVector3fGyro)

    def getAccelerometer(self):
        '''Gets the current state of the accelerometers in the module.

        Returns:

            accels (float, float, float): (x,y,z) accelerometer data, in
                                          m/s^2.'''

        return self._getVector3fField(Info.FeedbackVector3fAccelerometer)

    def getPosition(self):
        '''Gets the position of the module

        Returns:

            position (float): the position of the angle of the module'''
        if not hebi.hebiFeedbackHasHighResAngle(
                self.getAddress(),
                Info.FeedbackHighResAnglePosition):
            print('No angle found')
            return None
        ipart = ctypes.pointer(ctypes.c_int())
        fpart = ctypes.pointer(ctypes.c_float())
        hebi.hebiFeedbackGetHighResAngle(self.getAddress(),
                                         Info.FeedbackHighResAnglePosition,
                                         ipart,
                                         fpart)
        return (ipart.contents.value, fpart.contents.value)

    def getPositionCommand(self):
        '''Gets the commanded position of the module

        Returns:

            position (float): the commanded position of the angle of the
                              module.
        '''
        if not hebi.hebiFeedbackHasHighResAngle(
                self.getAddress(),
                Info.FeedbackHighResAnglePositionCommand):
            print('No angle found')
            return None
        ipart = ctypes.pointer(ctypes.c_int())
        fpart = ctypes.pointer(ctypes.c_float())
        hebi.hebiFeedbackGetHighResAngle(
            self.getAddress(),
            Info.FeedbackHighResAnglePositionCommand,
            ipart,
            fpart
        )
        return (ipart.contents.value, fpart.contents.value)

    def getTorque(self):
        '''Gets the torque on the module

        Returns:

            torque (float): the torque felt on the module'''
        return self._getField(Info.FeedbackFloatTorque)

    def getBoardTemperature(self):
        '''Gets the board temperature of the module

        Returns:

            temp (float): Ambient temperature inside the module (measured at
                          the IMU chip), in degrees Celsius.'''
        return self._getField(Info.FeedbackFloatBoardTemperature)

    def getProcessorTemperature(self):
        '''Gets the processor temperature of the module.

        Returns:

            temp (float): Temperature of the processor chip, in degrees
                          Celsius.
        '''
        return self._getField(Info.FeedbackFloatProcessorTemperature)

    def getVoltage(self):
        '''Gets the voltage of the bus.

        Returns:

            volts (float): Bus voltage that the module is running at (in
                           Volts).'''
        return self._getField(Info.FeedbackFloatVoltage)

    def getVelocity(self):
        '''Gets the velocity of the module.

        Returns:

            velocity (float): Velocity of the module output (post-spring), in
                              radians/second.'''
        return self._getField(Info.FeedbackFloatVelocity)

    def getVelocityCommand(self):
        '''Gets the commanded velocity of the module.

        Returns:

            commandedVelocity (float): Commanded velocity of the module output
                                       (post-spring), in radians/second.'''
        return self._getField(Info.FeedbackFloatVelocityCommand)

    def getTorqueCommand(self):
        '''Gets the commanded torque of the module.

        Returns:

            commandedTorque (float): Commanded torque of the module output, in
                                     N * m.'''
        return self._getField(Info.FeedbackFloatTorqueCommand)

    def getDeflection(self):
        '''Gets the deflection of the spring in the module.

        Returns:

            deflection (float): Difference (in radians) between the pre-spring
                                and post-spring output position.'''
        return self._getField(Info.FeedbackFloatDeflection)

    def getDeflectionVelocity(self):
        '''Gets the velocity of the deflection of the spring in the module.

        Returns:

            deflectionVelocity (float): Velocity (in radians/second) of the
                                        difference between the pre-spring and
                                        post-spring output position.'''
        return self._getField(Info.FeedbackFloatDeflectionVelocity)

    def getMotorVelocity(self):
        '''Gets the velocity of the motor.

        Returns:

            motorVelocity (float): The velocity (in radians/second) of the
                                   motor shaft.'''
        return self._getField(Info.FeedbackFloatMotorVelocity)

    def getMotorCurrent(self):
        '''Gets the current to the motor.

        Returns:

            current (float): Current supplied to the motor.'''
        return self._getField(Info.FeedbackFloatMotorCurrent)

    def getMotorSensorTemperature(self):
        '''Gets temperature of the motor.

        Returns:

            temp (float): The temperature from a sensor near the motor housing
                          (in Celsius).'''
        return self._getField(Info.FeedbackFloatMotorSensorTemperature)

    def getMotorWindingCurrent(self):
        '''Gets the current in the motor windings

        Returns:

            current (float): The estimated current in the motor windings'''
        return self._getField(Info.FeedbackFloatMotorWindingCurrent)

    def getMotorWindingTemperature(self):
        '''Gets the temperature of the motor windings.

        Returns:

            temperature (float): the estimated temperature of the motor
                                 windings (in Celsius).'''
        return self._getField(Info.FeedbackFloatMotorWindingTemperature)

    def getMotorHousingTemperature(self):
        '''Gets the temperature of the motor housing.

        Returns:

            temperature (float): The estimated temperature of the motor
                                 housing.'''
        return self._getField(Info.FeedbackFloatMotorHousingTemperature)


class HebiGroupFeedback(HebiPointer):
    '''Object used to encapsulate feedback for a HebiGroup. Should not be
    directly created.

    Attributes:

        numModules (int): the number of modules represented by this
                          HebiGroupFeedback

    Args:

        create (bool): whether to initialze  automatically
        addr    (int): the initialzation address. Required if create == False,
                       ignored if create == True.
        '''
    def __init__(self, numModules, create=True, addr=0):
        if create:
            addr = hebi.hebiGroupFeedbackCreate(numModules)
        self.numModules = numModules
        HebiPointer.__init__(self, addr)

    def _close(self, addr):
        hebi.hebiGroupFeedbackRelease(addr)

    def _getFields(self, field):
        feedbacks = self._getFeedbackList()
        values = []
        for feedback in feedbacks:
            # pylint: disable=protected-access
            values.append(feedback._getField(field))
        return values

    def _getVector3fFields(self, field):
        feedbacks = self._getFeedbackList()
        values = []
        for feedback in feedbacks:
            # pylint: disable=protected-access
            values.append(feedback._getVector3fField(field))
        return values

    def _getFeedbackList(self):
        fbkList = []
        for i in range(self.numModules):
            fbk = HebiFeedback(
                hebi.hebiGroupFeedbackGetModuleFeedback(self.getAddress(), i)
            )
            fbkList.append(fbk)
        return fbkList

    def getFields(self, field):
        '''Gets the float for a field for every module in the group.

        Args:

            field (int): the field desired (in HebiConstants.Info)

        Returns:

            values (float list): a 1 x n numpy array of the associated
                                 values.
        '''
        return returnArray(self._getFields(field))

    def requestFeedback(self, group, timeout=15, wait=True):
        '''Requests feedback from the group and stores it into this object.

        Args:

            group (HebiGroup): the group to request feedback from
            timeout     (int): time, in milliseconds before timing out on the
                               feedback request
            wait       (bool): if True, will block code until feedback is
                               received.'''
        while hebi.hebiGroupRequestFeedback(group.getAddress(),
                                            self.getAddress(),
                                            timeout) != 0 and not wait:
            pass

    def getFeedbackList(self):
        '''Gets the feedback for each module in the group.

        Returns:

            feedbackList (HebiFeedback list): a list of HebiFeedbacks which
                                              are tied to each module in the
                                              group.
        '''
        return returnArray(self._getFeedbackList())

    def getGyros(self):
        '''Gets the gyroscope data for each module in the group.

        Returns:

            gyros ((float tuple) list): a list of 3x1 tuples of the x,y,z data
                                        for each module.
        '''
        return returnArray(self._getVector3fFields(Info.FeedbackVector3fGyro))

    def getAccelerometers(self):
        '''Gets the accelerometer data for each module in the group.

        Returns:

            accels ((float tuple) list): a list of 3x1 tuples of the x,y,z data
                                        for each module.

        '''
        return returnArray(self._getVector3fFields(Info.FeedbackVector3fAccelerometer))

    def getTorques(self):
        '''Gets the torques on the modules

        Returns:

            torque (float list): the torques felt on the modules'''
        return returnArray(self._getFields(Info.FeedbackFloatTorque))

    def getBoardTemperatures(self):
        '''Gets the board temperatures of the modules

        Returns:

            temp (float list): Ambient temperature inside the module
                                      (measured at the IMU chip), in degrees
                                      Celsius.
        '''
        return returnArray(self._getFields(Info.FeedbackFloatBoardTemperature))

    def getProcessorTemperatures(self):
        '''Gets the processor temperatures of the modules.

        Returns:

            temp (float list): Temperature of the processor chip, in
                                      degrees Celsius.
        '''
        return returnArray(self._getFields(
            Info.FeedbackFloatProcessorTemperature
        ))

    def getVoltages(self):
        '''Gets the voltages of the buses.

        Returns:

            volts (float list): Bus voltage that the module is running
                                at (in Volts).'''
        return returnArray(self._getFields(Info.FeedbackFloatVoltage))

    def getVelocities(self):
        '''Gets the velocities of the modules.

        Returns:

            velocities (float list): Velocities of the module outputs
                                     (post-spring), in radians/second.
        '''
        return returnArray(self._getFields(Info.FeedbackFloatVelocity))

    def getVelocityCommands(self):
        '''Gets the commanded velocities of the module.

        Returns:

            commandedVelocities (float list): Commanded velocities of
                                the module outputs (post-spring), in
                                radians/second.
        '''
        return returnArray(self._getFields(Info.FeedbackFloatVelocityCommand))

    def getTorqueCommands(self):
        '''Gets the commanded torques of the modules.

        Returns:

            commandedTorque (float list): Commanded torques of the
                                          module outputs, in N * m.
        '''
        return returnArray(self._getFields(Info.FeedbackFloatTorqueCommand))

    def getDeflections(self):
        '''Gets the deflections of the springs in the modules.

        Returns:

            deflections (float list): Difference (in radians) between
                                      the pre-spring and post-spring
                                      output position.
        '''
        return returnArray(self._getFields(Info.FeedbackFloatDeflection))

    def getDeflectionVelocities(self):
        '''Gets the velocities of the deflections of the springs in the modules.

        Returns:

            deflectVelocities (float list): Velocities (in radians/second) of
                                            the difference between the pre-
                                            spring and post-spring output
                                            positions.
        '''
        return returnArray(self._getFields(
            Info.FeedbackFloatDeflectionVelocity
        ))

    def getMotorVelocities(self):
        '''Gets the velocities of the motor.

        Returns:

            motorVelocities (float list): The velocities (in radians/second)
                                          of the motor shaft.'''
        return returnArray(self._getFields(Info.FeedbackFloatMotorVelocity))

    def getMotorCurrents(self):
        '''Gets the currents to the motors.

        Returns:

            currents (float list): Currents supplied to the motors.'''
        return returnArray(self._getFields(Info.FeedbackFloatMotorCurrent))

    def getMotorSensorTemperatures(self):
        '''Gets temperatures of the motors.

        Returns:

            temps (float list): The temperatures from sensors near the motor
                                housings (in Celsius).
        '''
        return returnArray(self._getFields(
            Info.FeedbackFloatMotorSensorTemperature
        ))

    def getMotorWindingCurrents(self):
        '''Gets the current in the motor windings

        Returns:

            current (float list): The estimated current in the motor windings.
        '''
        return returnArray(self._getFields(
            Info.FeedbackFloatMotorWindingCurrent
        ))

    def getMotorWindingTemperatures(self):
        '''Gets the temperature of the motor windings.

        Returns:

            temperature (float list): the estimated temperature of the motor
                                      windings (in Celsius).'''
        return returnArray(self._getFields(
            Info.FeedbackFloatMotorWindingTemperature
        ))

    def getMotorHousingTemperatures(self):
        '''Gets the temperatures of the motor housings.

        Returns:

            temperature (float list): The estimated temperatures of the motor
                                      housings.
        '''
        return returnArray(self._getFields(
            Info.FeedbackFloatMotorHousingTemperature
        ))


class HebiGroupInfo(HebiPointer):
    '''Object used to encapsulate info for a HebiGroup. Should not be direcly
    created.

    Attributes:

        numModules (int): the number of modules represented by this
                          HebiGroupInfo

    Args:

        create (bool): whether to initialze  automatically
        addr    (int): the initialzation address. Required if create == False,
                       ignored if create == True.'''
    def __init__(self, numModules, create=True, addr=0):
        if create:
            addr = hebi.hebiGroupInfoCreate(numModules)
        self.numModules = numModules
        HebiPointer.__init__(self, addr)

    def _getInfoList(self):
        fbkList = []
        for i in range(self.numModules):
            fbk = HebiInfo(hebi.hebiGroupInfoGetModuleInfo(self.getAddress(),
                                                           i))
            fbkList.append(fbk)
        return fbkList

    def requestInfo(self, group, timeout=15):
        '''Requests info from the group and stores it into this object.

        Args:

            group (HebiGroup): the group to request info from
            timeout     (int): time, in milliseconds before timing out on the
                               feedback request.
        '''
        hebi.hebiGroupRequestInfo(group.getAddress(),
                                  self.getAddress(),
                                  timeout)

    def getNames(self):
        '''Gets a list of all the names of the modules in the group

        Returns:

            names (str list): the names of every module in the group'''
        return returnArray([info.getName() for info in self._getInfoList()])

    def getFamilies(self):
        '''Gets a list of all the Families of the modules in the group

        Returns:

            families (str list): the families of every module in the group.
        '''
        return returnArray([info.getFamily() for info in self._getInfoList()])

    def getInfoList(self):
        '''Gets the info for each module in the group

        This blocks code until complete Info is returned.

        Returns:

            infoList (HebiInfo list): a list of HebiInfos which are tied to
                                      each module in the group.
        '''
        return returnArray(self._getInfoList())
