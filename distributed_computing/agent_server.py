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
from recognize_posture import PostureRecognitionAgent
from inverse_kinematics import InverseKinematicsAgent
from xmlrpc.server import SimpleXMLRPCServer
import threading
import os
import sys
sys.path.append(os.path.join(os.path.abspath(
    os.path.dirname(__file__)), '..', 'kinematics'))


class ServerAgent(InverseKinematicsAgent, PostureRecognitionAgent):

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.perception.joint[joint_name]

    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.target_joints[joint_name] = angle

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.recognize_posture(self.perception)

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
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
        while not current_time - start < higestTime:
            now = time.time()
        return True

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        transform = self.transforms[name]
        returnValue = transform
        transformLength = len(transform)
        for i in range(transformLength):
            specificLength = len(transform[i])
            for j in range(specificLength):
                returnValue[i][j] = float(transform[i][j])
        return returnValue

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.set_transforms()


def startServer():
    server = SimpleXMLRPCServer(('localhost', 8000))
    server.register_instance(ServerAgent(), allow_dotted_names=True)
    server.register_introspection_functions()
    server.register_multicall_functions()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, exiting.")
        sys.exit(0)


if __name__ == '__main__':
    agent = ServerAgent()
    thread=threading.Thread(target = start_server)
    thread.start()
    agent.run()
