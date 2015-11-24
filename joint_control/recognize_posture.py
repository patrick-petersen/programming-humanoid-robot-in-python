'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import hello
from keyframes import rightBackToStand

import pickle
from os import listdir, path
import numpy as np
from sklearn import svm, metrics


class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        ROBOT_POSE_CLF = 'robot_pose.pkl'
        self.posture_classifier = pickle.load(open(ROBOT_POSE_CLF))  # LOAD YOUR CLASSIFIER
        ROBOT_POSE_DATA_DIR = 'robot_pose_data'
        self.classes = listdir(ROBOT_POSE_DATA_DIR)

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE

        # YOUR CODE HERE
        # get angles of: ['AngleX', 'AngleY', 'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch']
        angles = []
        angles.append(perception.imu[0])
        angles.append(perception.imu[1])
        angles.append(perception.joint['LHipYawPitch'])
        angles.append(perception.joint['LHipRoll'])
        angles.append(perception.joint['LHipPitch'])
        angles.append(perception.joint['LKneePitch'])
        angles.append(perception.joint['RHipYawPitch'])
        angles.append(perception.joint['RHipRoll'])
        angles.append(perception.joint['RHipPitch'])
        angles.append(perception.joint['RKneePitch'])
        
        angles = np.array(angles).reshape(1, -1)
        
        posture = self.posture_classifier.predict(angles)[0]
        print self.classes[posture]

        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = rightBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
