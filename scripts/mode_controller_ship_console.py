#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
import numpy as np

buttons = np.zeros(30)
arming_value = True

def callback_joy(data):
    global buttons
    global arming_value

    rospy.wait_for_service('/mavros/set_mode')
    try:
        if data.buttons[15] == 1:
            if buttons[15] == 0:
                set_mode(0, 'manual')
                buttons[15] = data.buttons[15]
        else:
            buttons[15] = data.buttons[15]

        if data.buttons[16] == 1:
            if buttons[16] == 0:
                set_mode(0, 'acro')
                buttons[16] = data.buttons[16]
        else:
            buttons[16] = data.buttons[16]

        if data.buttons[17] == 1:
            if buttons[17] == 0:
                set_mode(0, 'auto')
                buttons[17] = data.buttons[17]
        else:
            buttons[17] = data.buttons[17]

        if data.buttons[18] == 1:
            if buttons[18] == 0:
                arming(not(arming_value))
                buttons[18] = data.buttons[18]
                arming_value = not(arming_value)
        else:
            buttons[18] = data.buttons[18]

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == '__main__':
    rospy.init_node('mode_controller')
    rospy.Subscriber("/joy", Joy, callback_joy)
    set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    rospy.spin()