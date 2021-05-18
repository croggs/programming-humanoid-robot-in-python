'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import leftBackToStand
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import *

class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.startTime = -1

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        (joint_names, times, keys) = keyframes
        if keyframes == ([], [], []):
            return target_joints

        if self.startTime == -1:
            self.startTime = perception.time

        current_time = perception.time - self.startTime



        known_joints = 0
        joint=0
        for joint_name in (joint_names):
            joint_time = 0

            if current_time > times[joint][-1]:
                known_joints += 1
                if known_joints == len(joint_names):
                    self.startTime = -1
                    self.keyframes = ([], [], [])
                continue
            upper_threshold = 0
            lower_threshold = 0
            for j in range(len(times[joint])):
                upper_threshold = times[joint][j]
                if ((current_time >= lower_threshold and current_time <= upper_threshold)):
                    joint_time = j
                    break
                lower_threshold = upper_threshold


            if joint_time == 0:
                p0 = 0
                p1 = 0
                p3 = keys[joint][joint_time][0]
                p2 = p3 + keys[joint][joint_time][1][2]
            else:
                p0 = keys[joint][joint_time - 1][0]
                p3 = keys[joint][joint_time][0]
                p1 = p0 + keys[joint][joint_time - 1][2][2]
                p2 = p3 + keys[joint][joint_time][1][2]
            t = (current_time - lower_threshold) / (upper_threshold - lower_threshold)
            term1 = np.power(1-t,3) * p0
            term2 = 3*t*np.power(1-t,2) * p1
            term3 = 3* np.power(t,2)*(1-t)*p2
            term4 = np.power(t, 3)*p3

            target_joints[joint_name] = term1 + term2 + term3 + term4

            if (joint_name == "LHipYawPitch"):
                target_joints["RHipYawPitch"] = term1 + term2 + term3 + term4
            joint +=1

        return target_joints
if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = leftBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
