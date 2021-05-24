'''In this exercise you need to implemente inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinemtatics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinemtatics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinemtatics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
import numpy as np
from math import atan2, sqrt, pi


class InverseKinematicsAgent(ForwardKinematicsAgent):

    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        # get angels for chains of effector
        minimumError = 1e-3
        chainsAngles = {key: self.perception.joint[key]
                        for key in self.chains[effector_name]}
        # end effector for the given effectr chain
        chainsOfEffector = self.chains[effector_name]
        # get last effector chain
        lastEffectorChain = chainsOfEffector[-1]
        # target values
        target = self.extractValues(transform, minimumError)
        # initalise angelsToReturn
        angelsToReturn = np.zeros(len(chainsAngles))

        # break loop after 1000 iteration or if angles are satisfied
        for x in range(1000):
            # execute forward kinematics
            self.forward_kinematics(chainsAngles)
            # save result of forward kinematics for joints
            fkResult = self.transforms
            # distance error between target and actual result of last Chain
            error = target - \
                self.extractValues(fkResult[lastEffectorChain], minimumError)
            # calculate jacobian Matrix
            jacobianMatrix = self.calcJacobianMatrix(
                target, fkResult, chainsOfEffector)

            # use jacobian transponse methode
            jacobianTE = np.dot(
                np.dot(jacobianMatrix, np.transpose(jacobianMatrix)), error)
            scalar = float(np.inner(error, jacobianTE)) / \
                float(np.inner(jacobianTE, jacobianTE))
            newAngles = scalar * np.dot(jacobianMatrix.T, error)

            # increase angelsToReturn by new angles
            angelsToReturn += newAngles
            # add updates angles to the angels of the chains
            for i, key in enumerate(chainsAngles.keys()):
                chainsAngles[key] += angelsToReturn[i]
            # check if error is low enougth
            if np.inner(error, error) < minimumError:
                break

        return angelsToReturn

    def calcJacobianMatrix(self, target, fkResult, chainsOfEffector):
        jacobianMatrix = np.zeros((6, len(chainsOfEffector)))
        for i, (x, t) in enumerate(fkResult.iteritems()):
            x, y, z = self.extractCoordinates(t)
            jacobianMatrix[: i] = (target - np.array([x, y, z, 0, 0, 0]))
        for i, joint in enumerate(chainsOfEffector):
            index = 0
            if 'Pitch' in joint:
                index = 3
            if 'Yaw' in joint:
                index = 4
            if 'Roll' in joint:
                index = 5
            jacobianMatrix[index, i] = 1
        return jacobianMatrix

    def extractCoordinates(self, transform):
        x = transform[-1, 0]
        y = transform[-1, 1]
        z = transform[-1, 2]

        return x, y, z

    def extractValues(self, transform, minimumError):
        """
        :param transform: transformation matrix
        :return array with coordinates and angles
        """

        x, y, z = self.extractCoordinates(transform)

        if(transform[2, 0] > minimumError):
            angleX = atan2(transform[2, 1], transform[2, 2])
            angleY = atan2(-transform[2, 0],
                           sqrt(transform[2, 1]**2 + transform[2, 2]**2))
            angleZ = atan2(transform[1, 0], transform[0, 0])
        else:
            angleZ = 0
            if(abs(transform[2, 0] + 1) < minimumError):
                angleX = atan2(transform[0, 1], transform[0, 2])
                angleY = pi / 2
            else:
                angleX = atan2(-transform[0, 1], -transform[0, 2])
                angleY = -pi / 2
        return np.array([x, y, z, angleX, angleY, angleZ])


    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        names = []
        times = []
        keys = []
        self.forward_kinematics(self.perception.joint)
        angles = self.inverse_kinematics(effector_name, transform)
        for i, joint in enumerate(self.chains[effector_name]):
            names.append(joint)
            times.append([1.0, 3.0])
            keys.append([[angles[i] - 0.01, [3, 0, 0], [3, 0, 0]],
                        [angles[i], [3, 0, 0], [3, 0, 0]]])
        # the result joint angles have to fill in
        self.keyframes = (names, times, keys)


if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = np.identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
