'''In this exercise you need to implement inverse kinematics for NAO's legs
* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''
from forward_kinematics import ForwardKinematicsAgent
import numpy as np
import math


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics
        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        #inverse
        joint_angles = {}
        joints = self.chains[effector_name]

        N = len(joints) # - 1
        theta = np.random.random(N) * 1e-5
        lambda_ = 1
        max_step = 0.1
        Ts_prep = []
        for joint in joints:
            Ts_prep.append(self.transforms[joint])
        for i in range(1000):
            self.forward_kinematics(self.perception.joint)
            Ts=Ts_prep

            Te = np.matrix([self.from_trans(Ts[-1])]).T
            print(Te)
            e = np.matrix([self.from_trans(transform)]) - Te
            e[e > max_step] = max_step
            e[e < -max_step] = -max_step
            T = np.matrix([self.from_trans(i) for i in Ts]).T
            J = Te - T
            dT = Te - T
            J[0, :] = -dT[1, :]# x
            J[1, :] = dT[0, :]# y
            J[-1, :] = 1# angular
            d_theta = lambda_ * np.linalg.pinv(J) * e
            theta += np.asarray(d_theta.T)[0]

            i = 0
            for joint in joints:
                joint_angles[joint] = theta[i]
                i = i + 1
                if(i==len(theta)):
                    break

            if np.linalg.norm(d_theta) < 1e-4:
                break
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        joint_names = self.chains[effector_name]
        times = []
        keys= []
        self.forward_kinematics(self.perception.joint)
        joint_angles = self.inverse_kinematics(effector_name, transform)
        for joint in range(len(joint_names)):
            keys.append([[self.perception.joint[joint_names[joint]], [3, 0, 0], [3, 0, 0]], [joint_angles[joint_names[joint]],[3, 0, 0], [3, 0, 0]]])

        for i in range(len(joint_names)):
            times.append([1, 5.0])

        self.keyframes = (joint_names, times, keys)

    def from_trans(self, matrix):
        x, y, z = matrix[3, 0], matrix[3, 1], matrix[3, 2]
        x2, y2, z2 = 0, 0, 0
        if T[0, 0] == 1:
            x2 = math.atan2(T[2, 1], T[1, 1])
        elif T[1, 1] == 1:
            y2 = math.atan2(T[0, 2], T[0, 0])
        elif T[2, 2] == 1:
            z2 = math.atan2(T[1, 0], T[0, 0])
        return np.array([x, y, z, x2, y2, z2])


if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = np.identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.3
    T[-1, 3] = 10
    agent.set_transforms('LLeg', T)
    agent.run()