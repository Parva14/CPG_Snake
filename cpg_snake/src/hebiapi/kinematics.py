import numpy as np
import ctypes

from hebiapi.base import hebi, HebiPointer, flatten, HebiAccessError
from hebiapi import returnsNumpyArrays
from hebiapi.constants import Kinematics as _K

FrameTypes = _K

_MATRIX = ctypes.c_float * 16
_TRANSFORM = lambda: ctypes.pointer(_MATRIX())

def returnMatrix(matrix):
    if returnsNumpyArrays():
        return np.array(matrix)
    else:
        return matrix

class HebiKinematicsTree(HebiPointer):

    def __init__(self, *args):
        if args:
            super().__init__(args[0])
        else:
            super().__init__(hebi.hebiKinematicsCreate())
        self.numBodies = 0

    def _close(self, addr):
        hebi.hebiKinematicsRelease(addr)

    def setBaseFrame(self, transform):
        '''Sets the fixed transform from the origin to the first added body.

        Args:

            transform (float): A 4x4 homogeneous transform.
        '''
        transformPointer = ctypes.pointer(_MATRIX(*flatten(transform)))
        hebi.hebiKinematicsSetBaseFrame(self.getAddress(), transformPointer)

    def getBaseFrame(self):
        '''Retreives the fixed transform from the origin to the first added
        body.

        Returns:

            transform (float list list): A 4x4 homogeneous transform matrix
        '''
        transformPointer = _TRANSFORM()
        hebi.hebiKinematicsGetBaseFrame(self.getAddress(), transformPointer)
        return returnMatrix([transformPointer.contents[i:4+i] for i in range(0,16,4)])

    def getNumberOfFrames(self, frameType):
        '''Gets the number of frames in the forward kinematics.

        Note that this depends on the type of frame requested – for center of mass frames, there is one per added body; for output frames, there is one per output per body.

        Args:

            frameType (kinematics.FrameType enum): the type of frame to use
        '''
        return hebi.hebiKinematicsGetNumberOfFrames(self.getAddress(), frameType.value)

    def getNumberOfDoFs(self):
        '''Gets the number of degrees of freedom in the kinematic tree.

        This is equal to the number of actuators added.

        Returns:

            DoFs (int): the number of degrees of freedom.
        '''
        return hebi.hebiKinematicsGetNumberOfDoFs(self.getAddress())

    def getJacobians(self, frameType, positions):
        '''Generates the jacobian for each frame for the kinematic tree.

        Args:

            frameType (kinematics enum): Which type fo frame to consider.
            positions (float list): A list of joint positions equal in length to the number of DoFs of the kinematic tree.
        '''
        dofs = self.getNumberOfDoFs()
        frameSize = 6 * dofs
        positionsPointer = ctypes.pointer((len(positions) * ctypes.c_double)(*flatten(positions)))
        jacobiansPointer = ctypes.pointer(((self.getNumberOfFrames(frameType) * frameSize) * ctypes.c_float)())
        hebi.hebiKinematicsGetJacobians(self.getAddress(), frameType.value, positionsPointer, jacobiansPointer)
        jacobians = list(jacobiansPointer.contents)
        jacobians = [jacobians[i:i+frameSize] for i in range(0, len(jacobians), frameSize)]
        jacobians = [[frame[i:i+dofs] for i in range(0, len(frame), dofs)] for frame in jacobians][-1]
        return returnMatrix(jacobians)

    def getForwardKinematics(self, frameType, positions):
        '''Generates the forward kinematics for the kinematic tree.

        The order of the returned frames is in a depth-first tree.

        Args:

            frameType (kinematics enum): Which type of frame to consider.
            positions      (float list): A list of joint positions equal in length to the number of DoFs of the kinematic tree.
        '''
        framesP = ctypes.pointer((ctypes.c_float * (16 * self.getNumberOfFrames(frameType)))())
        positions = (ctypes.c_double * self.getNumberOfDoFs())(*flatten(positions))
        hebi.hebiKinematicsGetForwardKinematics(self.getAddress(), frameType.value, positions, framesP)
        frames = list(framesP.contents)
        return returnMatrix([[frames[i:i+16][j:4+j] for j in range(0,16,4)] for i in range(0, len(frames), 16)])

    def addBody(self, body, parentBody):
        '''Add a body to a parent body connected to a kinematic tree.

        The parent body must be on the kinematic tree. To attach the initial body to the kinematics object, use None for the parentBody argument.

        Args:

            body (kinematics.HebiBody): The body to add to the tree.
            parentBody (kinematics.HebiBody): The body to attach to on the tree.
        '''
        if parentBody is None and self.numBodies > 0:
            raise HebiAccessError('Cannot add another root body to tree')
        if parentBody is not None:
            if parentBody.child is not None:
                raise HebiAccessError('Only one output allowed per body')
            if not parentBody.onTree:
                raise HebiAccessError('Parent body must be on the tree.')
            parentAddress = parentBody.getAddress()
        else:
            parentAddress = 0
        if body.onTree:
            raise HebiAccessError('Body is already on the tree.')
        if hebi.hebiKinematicsAddBody(self.getAddress(), parentAddress, 0, body.getAddress()) != 0:
            raise HebiAccessError('Something went wrong adding the body to the tree')
        body.onTree = True
        self.numBodies += 1
        if parentBody is not None:
            parentBody.child = body


class _HebiBody(HebiPointer):
    def __init__(self, addr):
        self.onTree = False
        self.child = None
        super().__init__(addr)
        if addr == 0:
            raise HebiAccessError('Could not create body')

    def _close(self, addr):
        if not self.onTree:
            hebi.hebiBodyRelease(addr)


class HebiStaticBody(_HebiBody):

    def __init__(self, centerOfMass, outputs):
        centerOfMassP = ctypes.pointer((ctypes.c_float * 3)(*centerOfMass))
        outputsP = ctypes.pointer((ctypes.c_float * (16 * len(outputs)))(*flatten(outputs)))
        addr = hebi.hebiStaticBodyCreate(centerOfMassP, len(outputs), outputsP)
        super().__init__(addr)


class HebiActuator(_HebiBody):
    def __init__(self, centerOfMass, inputToJoint, jointRotationAxis, jointToOutput):
        centerOfMassP = (ctypes.c_float * 3)(*centerOfMass)
        inputToJointP = _MATRIX(*flatten(inputToJoint))
        jointRotationAxisP = (ctypes.c_float * 3)(*jointRotationAxis)
        jointToOutputP = _MATRIX(*flatten(jointToOutput))
        addr = hebi.hebiActuatorCreate(centerOfMassP, inputToJointP, jointRotationAxisP, jointToOutputP)
        super().__init__(addr)

