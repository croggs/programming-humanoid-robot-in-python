'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref
import xmlrpc.client as xmlrpclib
import threading
from joint_control.keyframes import hello
from numpy import identity
import numpy as np


class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''

    def __init__(self, obj):
        self.obj = obj
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE
        self.obj.clients.execute_keyframes(keyframes)

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE
        self.obj.clients.set_transform(effector_name, transform)


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''

    # YOUR CODE HERE
    def __init__(self):
        self.post = PostHandler(self)
        self.clients = xmlrpclib.ServerProxy("http://localhost:8080/")
        print(self.clients)

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.clients.get_angle(joint_name)

    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.clients.set_angle(joint_name, angle)

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.clients.get_posture()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.post.execute_keyframes(keyframes)

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE

        return self.clients.get_transform(name)

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.clients.set_transform(effector_name, transform)


if __name__ == '__main__':
    agent = ClientAgent()
    # TEST CODE HERE
    #agent.set_angle('HeadYaw', 1)
    #print(agent.get_angle('HeadYaw'))
    #agent.execute_keyframes(hello())
    #print("keyframe")
    #print(agent.get_posture())
    print(agent.get_transform('LHipYawPitch'))
    T = [[1, 0, 0, 0],
         [0, 1, 0, 0.5],
         [0, 0, 1, 0.26],
         [0, 0, 0, 1]]
    print(agent.set_transform('LLeg', T))
