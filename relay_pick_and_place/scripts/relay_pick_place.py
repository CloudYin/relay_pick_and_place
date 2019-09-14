#!/usr/bin/env python
from geometry_msgs.msg import Pose, Point
from pilz_robot_programming import *
import math
import rospy
from relay_pick_and_place.srv import *

__REQUIRED_API_VERSION__ = "1"  # API version
__MOVE_VELOCITY__ = 0.3  # velocity of the robot
__PnP_VELOCITY__ = 0.01 # velocity of pick and place

def handle_pause_movement(req):
    if req.Pause:
        r.pause()
    else:
        r.resume()
    return PauseMovementResponse(req.Pause)


def pause_movement_server():
    s = rospy.Service('move_enable', PauseMovement, handle_pause_movement)
    # rospy.spin()


# main program
def start_program():
    pause_movement_server()
    rospy.loginfo("Program started")  # log

    # start position (unit: degree)
    # A1: 0
    # A2: 0
    # A3: 90
    # A4: 0
    # A5: 90
    # A6: 0
    home_pos = [0, 0, math.radians(90), 0, math.radians(90), math.radians(-45)]

    # transport position (unit: degree)
    # A1: 0
    # A2: -115
    # A3: -135
    # A4: 90
    # A5: 90
    # A6: 0
    transport_pos = [
        0,
        math.radians(-115),
        math.radians(-135),
        math.radians(90),
        math.radians(90),
        0,
    ]



    # Move to start point 
    r.move(Ptp(goal=home_pos, vel_scale=__MOVE_VELOCITY__))  
    r.move(Gripper(goal=0.02))

    while True:
        # small remove
        # small pick
        r.move(Ptp(goal=Pose(position=Point(-0.34, 0.08, 0.17), orientation=from_euler(0, math.radians(180), math.radians(-45))), vel_scale=__MOVE_VELOCITY__))
        r.move(Gripper(goal=0.015))

        # move down
        r.move(Lin(goal=Pose(position=Point(-0.34, 0.08, 0.135)), vel_scale=__PnP_VELOCITY__))
        r.move(Gripper(goal=0.010))

        # move up
        r.move(Lin(goal=Pose(position=Point(-0.34, 0.08, 0.17)), vel_scale=__PnP_VELOCITY__))
        r.move(Gripper(goal=0.010))

        # small place
        r.move(Lin(goal=Pose(position=Point(-0.041, -0.006, 0.17), orientation=from_euler(0, math.radians(180), math.radians(45))), vel_scale=__MOVE_VELOCITY__))
        r.move(Gripper(goal=0.010))

        # move down
        r.move(Lin(goal=Pose(position=Point(-0.041, -0.006, 0.14)), vel_scale=__PnP_VELOCITY__))
        r.move(Gripper(goal=0.015))

        # move up
        r.move(Lin(goal=Pose(position=Point(-0.041, -0.006, 0.17)), vel_scale=__PnP_VELOCITY__))
        r.move(Gripper(goal=0.015))

        # # Move to start point 
        # r.move(Ptp(goal=home_pos, vel_scale=__MOVE_VELOCITY__))  
        # r.move(Gripper(goal=0.02))

        # medium remove
        # # Move to start point 
        # r.move(Ptp(goal=home_pos, vel_scale=__MOVE_VELOCITY__))  
        # r.move(Gripper(goal=0.02))

        # medium pick
        r.move(Ptp(goal=Pose(position=Point(-0.33, 0.046, 0.17), orientation=from_euler(0, math.radians(180), math.radians(-45))), vel_scale=__MOVE_VELOCITY__))
        r.move(Gripper(goal=0.015))

        # move down
        r.move(Lin(goal=Pose(position=Point(-0.33, 0.046, 0.135)), vel_scale=__PnP_VELOCITY__))
        r.move(Gripper(goal=0.010))

        # move up
        r.move(Lin(goal=Pose(position=Point(-0.33, 0.046, 0.17)), vel_scale=__PnP_VELOCITY__))
        r.move(Gripper(goal=0.010))

        # medium place
        r.move(Lin(goal=Pose(position=Point(-0.041, 0.098, 0.17), orientation=from_euler(0, math.radians(180), math.radians(45))), vel_scale=__MOVE_VELOCITY__))
        r.move(Gripper(goal=0.010))

        # move down
        r.move(Lin(goal=Pose(position=Point(-0.041, 0.098, 0.14)), vel_scale=__PnP_VELOCITY__))
        r.move(Gripper(goal=0.015))

        # move up
        r.move(Lin(goal=Pose(position=Point(-0.041, 0.098, 0.17)), vel_scale=__PnP_VELOCITY__))
        r.move(Gripper(goal=0.015))

        # # Move to start point 
        # r.move(Ptp(goal=home_pos, vel_scale=__MOVE_VELOCITY__))  
        # r.move(Gripper(goal=0.02))


        # big remove
        # # Move to start point 
        # r.move(Ptp(goal=home_pos, vel_scale=__MOVE_VELOCITY__))  
        # r.move(Gripper(goal=0.02))

        # big pick
        r.move(Ptp(goal=Pose(position=Point(-0.33, -0.1086, 0.17), orientation=from_euler(0, math.radians(180), math.radians(-45))), vel_scale=__MOVE_VELOCITY__))
        r.move(Gripper(goal=0.025))

        # move down
        r.move(Lin(goal=Pose(position=Point(-0.33, -0.1086, 0.13)), vel_scale=__PnP_VELOCITY__))
        r.move(Gripper(goal=0.018))

        # move up
        r.move(Lin(goal=Pose(position=Point(-0.33, -0.1086, 0.17)), vel_scale=__PnP_VELOCITY__))
        r.move(Gripper(goal=0.018))

        # big place
        r.move(Lin(goal=Pose(position=Point(-0.042, -0.113, 0.17), orientation=from_euler(0, math.radians(180), math.radians(45))), vel_scale=__MOVE_VELOCITY__))
        r.move(Gripper(goal=0.018))

        # move down
        r.move(Lin(goal=Pose(position=Point(-0.042, -0.113, 0.134)), vel_scale=__PnP_VELOCITY__))
        r.move(Gripper(goal=0.025))

        # move up
        r.move(Lin(goal=Pose(position=Point(-0.042, -0.113, 0.17)), vel_scale=__PnP_VELOCITY__))
        r.move(Gripper(goal=0.025))

        # # Move to start point 
        # r.move(Ptp(goal=home_pos, vel_scale=__MOVE_VELOCITY__))  
        # r.move(Gripper(goal=0.02))


        # big install
        # # Move to start point 
        # r.move(Ptp(goal=home_pos, vel_scale=__MOVE_VELOCITY__))  
        # r.move(Gripper(goal=0.02))

        # big pick
        r.move(Ptp(goal=Pose(position=Point(-0.042, -0.113, 0.17), orientation=from_euler(0, math.radians(180), math.radians(45))), vel_scale=__MOVE_VELOCITY__))
        r.move(Gripper(goal=0.025))

        # move down
        r.move(Lin(goal=Pose(position=Point(-0.042, -0.113, 0.134)), vel_scale=__PnP_VELOCITY__))
        r.move(Gripper(goal=0.018))

        # move up
        r.move(Lin(goal=Pose(position=Point(-0.042, -0.113, 0.17)), vel_scale=__PnP_VELOCITY__))
        r.move(Gripper(goal=0.018))

        # big place
        r.move(Lin(goal=Pose(position=Point(-0.33, -0.1086, 0.17), orientation=from_euler(0, math.radians(180), math.radians(-45))), vel_scale=__MOVE_VELOCITY__))
        r.move(Gripper(goal=0.018))

        # move down
        r.move(Lin(goal=Pose(position=Point(-0.33, -0.1086, 0.13)), vel_scale=__PnP_VELOCITY__))
        r.move(Gripper(goal=0.025))

        # move up
        r.move(Lin(goal=Pose(position=Point(-0.33, -0.1086, 0.17)), vel_scale=__PnP_VELOCITY__))
        r.move(Gripper(goal=0.025))

        # # Move to start point 
        # r.move(Ptp(goal=home_pos, vel_scale=__MOVE_VELOCITY__))  
        # r.move(Gripper(goal=0.02))

        # medium install
        # # Move to start point 
        # r.move(Ptp(goal=home_pos, vel_scale=__MOVE_VELOCITY__))  
        # r.move(Gripper(goal=0.02))

        # medium pick
        r.move(Ptp(goal=Pose(position=Point(-0.041, 0.098, 0.17), orientation=from_euler(0, math.radians(180), math.radians(45))), vel_scale=__MOVE_VELOCITY__))
        r.move(Gripper(goal=0.015))

        # move down
        r.move(Lin(goal=Pose(position=Point(-0.041, 0.098, 0.135)), vel_scale=__PnP_VELOCITY__))
        r.move(Gripper(goal=0.010))

        # move up
        r.move(Lin(goal=Pose(position=Point(-0.041, 0.098, 0.17)), vel_scale=__PnP_VELOCITY__))
        r.move(Gripper(goal=0.010))

        # medium place
        r.move(Lin(goal=Pose(position=Point(-0.33, 0.046, 0.17), orientation=from_euler(0, math.radians(180), math.radians(-45))), vel_scale=__MOVE_VELOCITY__))
        r.move(Gripper(goal=0.010))

        # move down
        r.move(Lin(goal=Pose(position=Point(-0.33, 0.046, 0.133)), vel_scale=__PnP_VELOCITY__))
        r.move(Gripper(goal=0.015))

        # move up
        r.move(Lin(goal=Pose(position=Point(-0.33, 0.046, 0.17)), vel_scale=__PnP_VELOCITY__))
        r.move(Gripper(goal=0.015))

        # # Move to start point 
        # r.move(Ptp(goal=home_pos, vel_scale=__MOVE_VELOCITY__))  
        # r.move(Gripper(goal=0.02))


        # small install
        # # Move to start point 
        # r.move(Ptp(goal=home_pos, vel_scale=__MOVE_VELOCITY__))  
        # r.move(Gripper(goal=0.02))

        # small pick
        r.move(Ptp(goal=Pose(position=Point(-0.041, -0.006, 0.17), orientation=from_euler(0, math.radians(180), math.radians(45))), vel_scale=__MOVE_VELOCITY__))
        r.move(Gripper(goal=0.015))

        # move down
        r.move(Lin(goal=Pose(position=Point(-0.041, -0.006, 0.135)), vel_scale=__PnP_VELOCITY__))
        r.move(Gripper(goal=0.010))

        # move up
        r.move(Lin(goal=Pose(position=Point(-0.041, -0.006, 0.17)), vel_scale=__PnP_VELOCITY__))
        r.move(Gripper(goal=0.010))

        # small place
        r.move(Lin(goal=Pose(position=Point(-0.34, 0.08, 0.17), orientation=from_euler(0, math.radians(180), math.radians(-45))), vel_scale=__MOVE_VELOCITY__))
        r.move(Gripper(goal=0.010))

        # move down
        r.move(Lin(goal=Pose(position=Point(-0.34, 0.08, 0.135)), vel_scale=__PnP_VELOCITY__))
        r.move(Gripper(goal=0.015))

        # move up
        r.move(Lin(goal=Pose(position=Point(-0.34, 0.08, 0.17)), vel_scale=__PnP_VELOCITY__))
        r.move(Gripper(goal=0.015))

        # Move to start point 
        r.move(Ptp(goal=home_pos, vel_scale=__MOVE_VELOCITY__))  
        r.move(Gripper(goal=0.02))

if __name__ == "__main__":
    # init a rosnode
    rospy.init_node("robot_program_node")
    # initialisation
    r = Robot(__REQUIRED_API_VERSION__)  # instance of the robot

    # start the main program
    start_program()
