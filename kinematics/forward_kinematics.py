'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    the local_trans has to consider different joint axes and link parameters for different joints
'''

# add PYTHONPATH
from numpy import array, cos, sin
from numpy.matlib import matrix, identity
import numpy as np
import os
import sys
sys.path.append(os.path.join(os.path.abspath(
    os.path.dirname(__file__)), '..', 'joint_control'))
from angle_interpolation import AngleInterpolationAgent


class ForwardKinematicsAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(
            simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 
                    #    'LWristYaw',
                                # 'LHand'
                                ],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll',
                        # 'RWristYaw',
                                # 'RHand'
                                ],
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch',
                                'LAnkleRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll']
                       }
        self.jointOffsets = {'HeadYaw': [0, 0, 126.5, 1],
                             'HeadPitch': [0, 0, 0, 1],
                             'LShoulderPitch': [0, 98, 100, 1],
                             'LShoulderRoll': [0, 0, 0, 1],
                             'LElbowYaw': [105, 15, 0, 1],
                             'LElbowRoll': [0, 0, 0, 1],
                            #  'LWristYaw': [55.95, 0, 0, 1],
                             'RShoulderPitch': [0, -98, 100, 1],
                             'RShoulderRoll': [0, 0, 0, 1],
                             'RElbowYaw': [105, -15, 0, 1],
                             'RElbowRoll': [0, 0, 0, 1],
                            #  'RWristYaw': [55.95, 0, 0, 1],
                             'LHipYawPitch': [0, 50, -85, 1],
                             'LHipRoll': [0, 0, 0, 1],
                             'LHipPitch': [0, 0, 0, 1],
                             'LKneePitch': [0, 0, -100, 1],
                             'LAnkleRoll': [0, 0, 0, 1],
                             'LAnklePitch': [0, 0, -102.9, 1],
                             'RHipYawPitch': [0, -50, -85, 1],
                             'RHipRoll': [0, 0, 0, 1],
                             'RHipPitch': [0, 0, 0, 1],
                             'RKneePitch': [0, 0, -100, 1],
                             'RAnkleRoll': [0, 0, 0, 1],
                             'RAnklePitch': [0, 0, -102.9, 1]}

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def calculatePitch(self, sinus, cosinus, offsetArray):
        return matrix([
            [cosinus, 0, sinus, 0],
            [0, 1, 0, 0],
            [-1 * sinus, 0, cosinus, 0],
            offsetArray
        ])

    def calculateYaw(self, sinus, cosinus, offsetArray):
        return matrix([
            [cosinus, sinus, 0, 0],
            [-1 * sinus, cosinus, 0, 0],
            [0, 0, 1, 0],
            offsetArray
        ])

    def calculateRoll(self, sinus, cosinus, offsetArray):
        return matrix([
            [1, 0, 0, 0],
            [0, cosinus, -1 * sinus, 0],
            [0, sinus, cosinus, 0],
            offsetArray
        ])

    def calculateByCoorinate(self, jointName, angle):
        sinus = sin(angle)
        cosinus = cos(angle)
        offsetArray = self.jointOffsets[jointName]
        if 'Pitch' in jointName:
            return self.calculatePitch(sinus, cosinus, offsetArray)
        if 'Yaw' in jointName:
            return self.calculateYaw(sinus, cosinus, offsetArray)
        if 'Roll' in jointName:
            return self.calculateRoll(sinus, cosinus, offsetArray)

    def local_trans(self, jointName, angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        # YOUR CODE HERE
        T =  self.calculateByCoorinate(jointName, angle)

        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        T = identity(4)
        for joint in joints.keys():
            angle = joints[joint]
            Tl = self.local_trans(joint, angle)
            self.transforms[joint] = np.dot(T, Tl)


if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
