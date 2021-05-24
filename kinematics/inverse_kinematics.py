'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joincurrentValues_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the resulcurrentValues of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
import math
import numpy as np


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, nameOfEffector, transform):
        '''solve the inverse kinematics

        :param str nameOfEffector: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        jointAngles = {}
        # Getting the angle for all joincurrentValues of the effector
        for jointName in self.chains[nameOfEffector]:
            jointAngles[jointName] = self.perception.joint[jointName]
        # Get Target result
        targetValues = self.getValues(transform.T)
        lambda_ = 0.002
        print('targetValues:', targetValues)
        while True:
            self.forward_kinematics(jointAngles)
            # get difference between target and calculated result
            currentValues = []
            for i, name in enumerate(self.chains[nameOfEffector]):
                currentValues.append(0)
                currentValues[i] = self.transforms[name]
            lastEffector = np.array(self.getValues(currentValues[-1]))
            print('currentValues', currentValues)
            error = targetValues - lastEffector
            print('ERROR', error)
            newValues = np.array([self.getValues(k) for k in currentValues[:]])
            jocobian = lastEffector - newValues
            print('Jacobian', jocobian)
            jocobian[:, -1] = 1
            newTheta = np.dot(np.dot(jocobian, np.linalg.pinv(
                np.dot(jocobian.T, jocobian))), error.T) * lambda_
            print('newTheata', newTheta)
            for i, name in enumerate(self.chains[nameOfEffector]):
                jointAngles[name] += np.asarray(newTheta)[i]
            print('UpdatedAngels:', jointAngles)
            if np.linalg.norm(newTheta) < 1e-3:
                break
        # YOUR CODE HERE

        return jointAngles

    def getValues(self, matrix):
        x = matrix[0, 3]
        y = matrix[1, 3]
        z = matrix[2, 3]

        t_x = math.atan2(matrix[2, 1], matrix[2, 2])
        t_y = math.atan2(-matrix[2, 0], math.sqrt(matrix[2, 1]
                         * matrix[2, 1] + matrix[2, 2] * matrix[2, 2]))
        t_z = math.atan2(matrix[1, 0], matrix[0, 0])

        return np.array([x, y, z, t_x, t_y, t_z])

    def updateAngles(self, joinCurrentValues, theta):

        return joincurrentValues

    def set_transforms(self, nameOfEffector, transform):
        '''solve the inverse kinematics and control joincurrentValues use the resulcurrentValues
        '''
        # YOUR CODE HERE
        # the result joint angles have to fill in
        names = []
        times = []
        keys = []
        self.forward_kinematics(self.perception.joint)
        angles = self.inverse_kinematics(nameOfEffector, transform)
        print(angles)

        for i, joint in enumerate(self.chains[nameOfEffector]):
            names.append(joint)
            times.append([1.0, 3.0])
            keys.append([[angles[joint] - 0.01, [3, 0, 0], [3, 0, 0]], [angles[joint], [3, 0, 0], [3, 0, 0]]]) 
        self.keyframes = (names, times, keys) # the result joint angles have to fill in
        print(self.keyframes)


if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
