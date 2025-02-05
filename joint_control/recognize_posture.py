'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import wipe_forehead
import numpy as np
import pickle
from os import listdir, path
ROBOT_POSE_CLF = 'robot_pose.pkl'



class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.class_postures = listdir(r'C:\Users\lucam\Desktop\dev\programming-humanoid-robot-in-python\joint_control\robot_pose_data')
        self.posture_classifier = pickle.load(open(r'C:\Users\lucam\Desktop\dev\programming-humanoid-robot-in-python\joint_control\robot_pose.pkl'))  # LOAD YOUR CLASSIFIER

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE
        data = []
        data.append(perception.joint['LHipYawPitch'])
        data.append(perception.joint['LHipRoll'])
        data.append(perception.joint['LHipPitch'])
        data.append(perception.joint['LKneePitch'])
        data.append(perception.joint['RHipYawPitch'])
        data.append(perception.joint['RHipRoll'])
        data.append(perception.joint['RHipPitch'])
        data.append(perception.joint['RKneePitch'])
        # AngleX
        data.append(perception.imu[0])
        # AngleY
        data.append(perception.imu[1])
        
        data = np.array(data).reshape(1, -1)
        
        index = self.posture_classifier.predict(data)
        posture =  self.class_postures[index[0]]
        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = wipe_forehead()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
