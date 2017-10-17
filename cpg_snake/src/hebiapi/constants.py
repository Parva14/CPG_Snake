'''
## constants - globally used constants are defined here

Underlying HEBI API version and identifiers for working with
fields/enums/command values are found here
'''
from enum import Enum

API_VERSION = '0.16'


class Commands:  # TODO: Change data encapsulation to something better
    '''Enums for Command related things'''
    CommandHighResAnglePosition = 0
    CommandEnumControlStrategy = 0
    CommandFloatVelocity = 0
    CommandFloatTorque = 1
    CommandFloatPositionKp = 2
    CommandFloatPositionKi = 3
    CommandFloatPositionKd = 4
    CommandFloatPositionFeedForward = 5
    CommandFloatPositionDeadZone = 6
    CommandFloatPositionIClamp = 7
    CommandFloatPositionPunch = 8
    CommandFloatPositionMinTarget = 9
    CommandFloatPositionMaxTarget = 10
    CommandFloatPositionTargetLowpass = 11
    CommandFloatPositionMinOutput = 12
    CommandFloatPositionMaxOutput = 13
    CommandFloatPositionOutputLowpass = 14
    CommandFloatVelocityKp = 15
    CommandFloatVelocityKi = 16
    CommandFloatVelocityKd = 17
    CommandFloatVelocityFeedForward = 18
    CommandFloatVelocityDeadZone = 19
    CommandFloatVelocityIClamp = 20
    CommandFloatVelocityPunch = 21
    CommandFloatVelocityMinTarget = 22
    CommandFloatVelocityMaxTarget = 23
    CommandFloatVelocityTargetLowpass = 24
    CommandFloatVelocityMinOutput = 25
    CommandFloatVelocityMaxOutput = 26
    CommandFloatVelocityOutputLowpass = 27
    CommandFloatTorqueKp = 28
    CommandFloatTorqueKi = 29
    CommandFloatTorqueKd = 30
    CommandFloatTorqueFeedForward = 31
    CommandFloatTorqueDeadZone = 32
    CommandFloatTorqueIClamp = 33
    CommandFloatTorquePunch = 34
    CommandFloatTorqueMinTarget = 35
    CommandFloatTorqueMaxTarget = 36
    CommandFloatTorqueTargetLowpass = 37
    CommandFloatTorqueMinOutput = 38
    CommandFloatTorqueMaxOutput = 39
    CommandFloatTorqueOutputLowpass = 40
    CommandFloatSpringConstant = 41


class Info:
    '''Enums for Info/Feedback related things'''
    FeedbackVector3fAccelerometer = 0
    FeedbackVector3fGyro = 1
    InfoEnumControlStrategy = 1
    InfoStringName = 0
    InfoStringFamily = 1
    InfoFloatPositionKp = 0
    InfoFloatPositionKi = 1
    InfoFloatPositionKd = 2
    InfoFloatPositionFeedForward = 3
    InfoFloatPositionDeadZone = 4
    InfoFloatPositionIClamp = 5
    InfoFloatPositionPunch = 6
    InfoFloatPositionMinTarget = 7
    InfoFloatPositionMaxTarget = 8
    InfoFloatPositionTargetLowpass = 9
    InfoFloatPositionMinOutput = 10
    InfoFloatPositionMaxOutput = 11
    InfoFloatPositionOutputLowpass = 12
    InfoFloatVelocityKp = 13
    InfoFloatVelocityKi = 14
    InfoFloatVelocityKd = 15
    InfoFloatVelocityFeedForward = 16
    InfoFloatVelocityDeadZone = 17
    InfoFloatVelocityIClamp = 18
    InfoFloatVelocityPunch = 19
    InfoFloatVelocityMinTarget = 20
    InfoFloatVelocityMaxTarget = 21
    InfoFloatVelocityTargetLowpass = 22
    InfoFloatVelocityMinOutput = 23
    InfoFloatVelocityMaxOutput = 24
    InfoFloatVelocityOutputLowpass = 25
    InfoFloatTorqueKp = 26
    InfoFloatTorqueKi = 27
    InfoFloatTorqueKd = 28
    InfoFloatTorqueFeedForward = 29
    InfoFloatTorqueDeadZone = 30
    InfoFloatTorqueIClamp = 31
    InfoFloatTorquePunch = 32
    InfoFloatTorqueMinTarget = 33
    InfoFloatTorqueMaxTarget = 34
    InfoFloatTorqueTargetLowpass = 35
    InfoFloatTorqueMinOutput = 36
    InfoFloatTorqueMaxOutput = 37
    InfoFloatTorqueOutputLowpass = 38
    InfoFloatSpringConstant = 39
    FeedbackHighResAnglePosition = 0
    FeedbackHighResAnglePositionCommand = 1
    FeedbackFloatBoardTemperature = 0
    FeedbackFloatProcessorTemperature = 1
    FeedbackFloatVoltage = 2
    FeedbackFloatVelocity = 3
    FeedbackFloatTorque = 4
    FeedbackFloatVelocityCommand = 5
    FeedbackFloatTorqueCommand = 6
    FeedbackFloatDeflection = 7
    FeedbackFloatDeflectionVelocity = 8
    FeedbackFloatMotorVelocity = 9
    FeedbackFloatMotorCurrent = 10
    FeedbackFloatMotorSensorTemperature = 11
    FeedbackFloatMotorWindingCurrent = 12
    FeedbackFloatMotorWindingTemperature = 13
    FeedbackFloatMotorHousingTemperature = 14

defaultGains = {'ControlStrategy': [0],  # TODO: change name to DEFAULT_GAINS
                'PositionKp': [0.0],
                'PositionKi': [0.0],
                'PositionKd': [0.0],
                'PositionFeedForward': [0.0],
                'PositionDeadZone': [0.0],
                'PositionIClamp': [0.0],
                'PositionPunch': [0.0],
                'PositionMinTarget': [0.0],
                'PositionMaxTarget': [0.0],
                'PositionTargetLowpass': [0.0],
                'PositionMinOutput': [0.0],
                'PositionMaxOutput': [0.0],
                'PositionOutputLowpass': [0.0],
                'VelocityKp': [0.0],
                'VelocityKi': [0.0],
                'VelocityKd': [0.0],
                'VelocityFeedForward': [0.0],
                'VelocityDeadZone': [0.0],
                'VelocityIClamp': [0.0],
                'VelocityPunch': [0.0],
                'VelocityMinTarget': [0.0],
                'VelocityMaxTarget': [0.0],
                'VelocityTargetLowpass': [0.0],
                'VelocityMinOutput': [0.0],
                'VelocityMaxOutput': [0.0],
                'VelocityOutputLowpass': [0.0],
                'TorqueKp': [0.0],
                'TorqueKi': [0.0],
                'TorqueKd': [0.0],
                'TorqueFeedForward': [0.0],
                'TorqueDeadZone': [0.0],
                'TorqueIClamp': [0.0],
                'TorquePunch': [0.0],
                'TorqueMinTarget': [0.0],
                'TorqueMaxTarget': [0.0],
                'TorqueTargetLowpass': [0.0],
                'TorqueMinOutput': [0.0],
                'TorqueMaxOutput': [0.0],
                'TorqueOutputLowpass': [0.0],
                'SpringConstant': [0.0]}

class Kinematics(Enum):
    CenterOfMass = 0
    Output = 1
