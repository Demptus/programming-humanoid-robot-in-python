'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
from SimpleXMLRPCServer import SimpleXMLRPCServer
import threading
import numpy as np
import os
import time
import sys
sys.path.append(os.path.join(os.path.abspath(
    os.path.dirname(__file__)), '..', 'kinematics'))
from inverse_kinematics import InverseKinematicsAgent
from recognize_posture import PostureRecognitionAgent


class ServerAgent(InverseKinematicsAgent, PostureRecognitionAgent):

    def __init__(self):
        super(ServerAgent, self).__init__()
        server = SimpleXMLRPCServer(
            ('localhost', 9000), logRequests=True, allow_none=True)
        server.register_instance(self)
        server.register_introspection_functions()
        server.register_multicall_functions()
        thread = threading.Thread(target=server.serve_forever)
        thread.start()

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        print("\nget angel", self.perception.joint[joint_name])
        return self.perception.joint[joint_name]

    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        print("\nSet Angle")
        self.target_joints[joint_name] = angle
        return True

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        print("\nget posture", self.recognize_posture(self.perception))
        return self.recognize_posture(self.perception)

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        print("\nExecute keyframes")
        self.keyframes = keyframes
        self.angle_interpolation(keyframes, self.perception)
        timeOfKeyframes = keyframes[1]
        finished = False
        keyframesLength = len(timeOfKeyframes)
        timeToWait = max([max(timeArray) for timeArray in timeOfKeyframes])
        start = time.time()
        now = start
        print(now,start,timeToWait)
        while not now - start > timeToWait:
            now = time.time()
        print ('done with keyframes')
        return True

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        print("\nget Transfrom")
        return self.transforms[name].tolist()

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        print("\nSet Transfrom", effector_name, transform)
        self.set_transforms(effector_name, np.matrix(transform))
        return True


if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()
