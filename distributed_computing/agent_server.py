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
        print ('init')
        thread = threading.Thread(target=server.serve_forever)
        thread.start()

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        # YOUR CODE HERE
        print("\nget angel", self.perception.joint[joint_name])
        return self.perception.joint[joint_name]

    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        # YOUR CODE HERE
        print("\nSet Angle")
        self.target_joints[joint_name] = angle

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        # YOUR CODE HERE
        print("\nget posture", self.recognize_posture(self.perception))
        return self.recognize_posture(self.perception)

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        # YOUR CODE HERE
        print("\nExecute keyframes ")
        self.keyframes = keyframes
        self.angle_interpolation(keyframes, self.perception)
        timeOfKeyframes = keyframes[1]
        finished = False
        keyframesLength = len(timeOfKeyframes)
        timeArray
        for i in range(keyframesLength):
            timeArray[i] = timeOfKeyframes[i][-1]
        timeArray.sort()
        higestTime = timeArray[-1]

        start = time.time()
        while not now - start < higestTime:
            now = time.time()
        print ('done with keyframes')
        return True

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        print("\nget Transfrom")
        transform = self.transforms[name]
        returnValue = transform
        transformLength = len(transform)
        print ('trans length', transformLength)
        for i in range(transformLength):
            specificLength = len(transform[i])
            print ('specificLength', specificLength)
            for j in range(specificLength):
                returnValue[i][j] = float(transform[i][j])
        np.array(transform)
        print ('Done value is', returnValue)
        return returnValue

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        print("\nSet Transfrom")
        self.set_transforms()


def start_server(self):
    server = SimpleXMLRPCServer.SimpleXMLRPCServer(
        ('localhost', 9000), logRequests=True, allow_none=True)
    server.register_instance(self)
    server.register_introspection_functions()
    server.register_multicall_functions()
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    # print("\ninit server")
    # server = SimpleXMLRPCServer(('localhost', 8000))
    # print("\nresgister instance")
    # server.register_instance(ServerAgent(), allow_dotted_names=True)
    # print("\nresgister functions")
    # server.register_introspection_functions()
    # print("\nresgister multicall functions")
    # server.register_multicall_functions()
    # try:
    #     print("\nstart running server forever")
    #     server.serve_forever()
    # except KeyboardInterrupt:
    #     print("\nKeyboard interrupt received, exiting.")
    #     sys.exit(0)


if __name__ == '__main__':
    agent = ServerAgent()
    # thread=threading.Thread(target = start_server)
    # thread.start()
    agent.run()
