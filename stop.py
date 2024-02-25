import time, socket, json, pickle
import anki_vector
import time
import socket as skt
import os
# from geometry_msgs.msg import TransformStamped
import sys, math, time
import socket, struct, threading
# import rospy
# import message_filters
import numpy as np
from util import quaternion_to_euler
from NatNetClient import NatNetClient


class runRobot:
    def __init__(self):
        self.currState = 0
        self.currTime = 0
        self.nRobot = 1
        self.posX = [None] * self.nRobot
        self.posY = [None] * self.nRobot
        self.posTheta = [None] * self.nRobot


    def connectToVectors(self):
        # List of serial numbers of the robots in order
        serialList = ["007039c3","00702c90","00702e03"]
        serialList = ["00702e03"]
        realRobotList = []

        #Connect to each Vector and save object to a list
        for serialNum in serialList:
            realRobotList.append(anki_vector.Robot(serialNum))
            realRobotList[-1].connect()

        return realRobotList



if __name__ == '__main__':


    # ----------- to receive positions from Optitrack ----------- 
    def receive_rigid_body_frame(opti_id, position, rotation_quaternion):
        # Position and rotation received
        if opti_id == 3 and position != []:
            position = position
            # position += (robot.arm.status['pos'],)
            # position += (robot.lift.status['pos'],)
            # print(rotation_quaternion)
            # The rotation is in quaternion. We need to convert it to euler angles
            newRot = (rotation_quaternion[2],rotation_quaternion[0],rotation_quaternion[1],rotation_quaternion[3])
            rotx, roty, rotz = quaternion_to_euler(newRot)
            # Store the roll pitch and yaw angles (deg)
            rotations = (rotx, roty, rotz)

            print(rotations)
            print(position)

    clientAddress = "199.168.1.155"     # computer IP
    optitrackServerAddress = "199.168.1.164"

    # This will create a new NatNet client
    streaming_client = NatNetClient()
    streaming_client.set_client_address(clientAddress)
    streaming_client.set_server_address(optitrackServerAddress)
    streaming_client.set_use_multicast(True)
    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.rigid_body_listener = receive_rigid_body_frame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()

    #  --------------------------------- 

    


    #Number of robots
    nRobot = 1
    r = runRobot()
    realRobotList = r.connectToVectors()
    currT = time.time()
    t = time.time()-currT

    for i in range(nRobot):
        realRobotList[i].motors.set_wheel_motors(300, 300)

    time.sleep(2)
    realRobotList[0].motors.set_wheel_motors(0, 0)


