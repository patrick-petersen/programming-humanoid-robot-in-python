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
    the local_trans has to consider different joint axes and link parameters for differnt joints
'''

# add PYTHONPATH
import os
import sys
import math
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity

from angle_interpolation import AngleInterpolationAgent


class ForwardKinematicsAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                      # YOUR CODE HERE
                      #'LeftArmJoints' : ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'LHand'],
                      'LeftArmJoints' : ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],
                      #'RightArmJoints' : ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'RHand'],
                      'RightArmJoints' : ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'],
                      'PelvisJoints' : ['LHipYawPitch', 'RHipYawPitch'], #LHipYawPitch and RHipYawPitch are physically just one motor so they cannot be controlled independently. In case of conflicting orders, LHipYawPitch always takes the priority.
                      'LeftLegJoints' : ['LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                      'RightLegJoints' : ['RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll']
                       }
        #the coordination ar always to the named joint and are in mm
        #the joint from is torso if first in line or the prior in line 
        self.chainsCordination = {'HeadYaw':(0,0,126.5), 'HeadPitch': (0,0,0),
                      'LShoulderPitch':(0,98,100), 'LShoulderRoll': (0,0,0), 'LElbowYaw': (105,15,0), 'LElbowRoll': (0,0,0), 'LWristYaw': (55.95, 0,0),
                      'RShoulderPitch':(0,-98,100), 'RShoulderRoll': (0,0,0), 'RElbowYaw': (105,15,0), 'RElbowRoll': (0,0,0), 'RWristYaw': (55.95, 0,0),
                      'LHipYawPitch': (0,50, -85),
                      'LHipRoll': (0,0,0), 'LHipPitch':(0,0,0), 'LKneePitch': (0,0,-100), 'LAnklePitch': (0,0,-102.9), 'LAnkleRoll' : (0,0,0),
                      'RHipYawPitch': (0,-50, -85),
                      'RHipRoll': (0,0,0), 'RHipPitch': (0,0,0), 'RKneePitch': (0,0,-100), 'RAnklePitch': (0,0,-102.9), 'RAnkleRoll': (0,0,0)
                       }


    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = matrix((4,4))
        # YOUR CODE HERE

        if joint_name.endswith("Pitch"):
            #X
            T = matrix([
                    [1, 0, 0, 0], 
                    [0, math.cos(joint_angle), - math.sin(joint_angle), 0], 
                    [0, math.sin(joint_angle), math.cos(joint_angle), 0], 
                    [0, 0, 0, 1]
                    ])
        elif joint_name.endswith("Yaw"):
            #Y
            T = matrix([
                    [math.cos(joint_angle), 0, math.sin(joint_angle), 0], 
                    [0, 1, 0, 0], 
                    [-math.sin(joint_angle),0, math.cos(joint_angle), 0], 
                    [0, 0, 0, 1]
                    ])
        elif joint_name.endswith("Roll"):
            #Z
            T = matrix([
                    [math.cos(joint_angle), -math.sin(joint_angle), 0, 0], 
                    [math.sin(joint_angle), math.cos(joint_angle), 0, 0], 
                    [0, 0, 1, 0], 
                    [0, 0, 0, 1]
                    ])
        else:
            print "Error!!"
            print joint_name

        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                T = T * Tl

                self.transforms[joint] = T * self.chainsCordination[joint]

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
