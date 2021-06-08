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
import os
import sys

sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent
from xmlrpc.server import SimpleXMLRPCServer
import threading
import numpy as np
import pickle




class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''

    # YOUR CODE HERE
    def __init__(self):
        print('Server Start')
        super(ServerAgent, self).__init__()
        self.posture = 'unknown'
        self.posture_classifier = pickle.load(open('../joint_control/robot_pose.pkl', 'rb'))

        self.server = SimpleXMLRPCServer(('0.0.0.0', 8080), allow_none=True)
        self.server.register_function(self.get_angle)
        self.server.register_function(self.set_angle)
        self.server.register_function(self.get_posture)
        self.server.register_function(self.execute_keyframes)
        self.server.register_function(self.get_transform)
        self.server.register_function(self.set_transform)
        self.thread = threading.Thread(target=self.server.serve_forever, args=[])
        self.thread.start()
        print("Server Aktiv")

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        print("get angle")
        return self.perception.joint[joint_name]

    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        print("set angle")
        self.target_joints[joint_name] = angle

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(ServerAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE
        data = [[perception.joint['LHipYawPitch'], perception.joint['LHipRoll'], perception.joint['LHipPitch'],
                perception.joint['LKneePitch'], perception.joint['RHipYawPitch'], perception.joint['RHipRoll'],
                perception.joint['RHipPitch'], perception.joint['RKneePitch'], perception.imu[0],
                perception.imu[1]]]


        posture = self.posture_classifier.predict(data)[0]

        return str(posture)

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        print("get posture")
        print(self.posture)
        return self.posture

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        print("execute keyframes")
        self.keyframes = keyframes

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        print("get_transform")
        return self.transforms[name]

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        print("set_transform")
        self.transforms[effector_name] = transform


if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()
