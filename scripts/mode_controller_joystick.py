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
        if data.buttons[2] == 1:
            if buttons[2] == 0:
                set_mode(0, 'manual')
                buttons[2] = data.buttons[2]
        else:
            buttons[2] = data.buttons[2]

        if data.buttons[3] == 1:
            if buttons[3] == 0:
                set_mode(0, 'acro')
                buttons[3] = data.buttons[3]
        else:
            buttons[3] = data.buttons[3]

        if data.buttons[1] == 1:
            if buttons[1] == 0:
                set_mode(0, 'auto')
                buttons[1] = data.buttons[1]
        else:
            buttons[1] = data.buttons[1]

        if data.buttons[0] == 1:
            if buttons[0] == 0:
                arming(not(arming_value))
                buttons[0] = data.buttons[0]
                arming_value = not(arming_value)
        else:
            buttons[0] = data.buttons[0]

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == '__main__':
    rospy.init_node('mode_controller')
    rospy.Subscriber("/joy", Joy, callback_joy)
    set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    rospy.spin()