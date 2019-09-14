#!/usr/bin/env python

import rospy
from relay_pick_and_place.srv import *
from pymodbus.client.sync import ModbusTcpClient


def read_pause_signal():
    client = ModbusTcpClient('169.254.60.1', port=502)  
    client.connect()

    result = client.read_coils(8224, 1)  # read(address, length)
    key_switch_status = result.bits[0]
    client.close()
    return key_switch_status

def pause_movement_client(key_switch):
    rospy.init_node('pause_movement_client')
    rospy.wait_for_service('move_enable')
    try:
        pause_movement = rospy.ServiceProxy('move_enable', PauseMovement)
        res = pause_movement(key_switch)
        return res.move_status
    except rospy.ServiceException:
        print("Service call failed.")

if __name__ == "__main__":
    while True:
        key_switch = read_pause_signal()
        pause_movement_client(key_switch)
