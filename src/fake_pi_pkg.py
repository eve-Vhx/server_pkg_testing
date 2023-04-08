#!/usr/bin/env python3
import rospy
from msg_pkg.srv import masterConnect


def run_fake_client():
    rospy.wait_for_service('drone_pi_connect_master')
    try:
        fake_pi_connect = rospy.ServiceProxy('drone_pi_connect_master', masterConnect)
        verification = fake_pi_connect('QROW11021')
        print("Successfully connected to server")
        print(verification)
    except rospy.ServiceException as e:
        print("fake pi connect service call failed")
    rospy.wait_for_service('nest_pi_connect_master')
    try:
        fake_pi_connect = rospy.ServiceProxy('nest_pi_connect_master', masterConnect)
        verification = fake_pi_connect('NEST11014')
        print("Successfully connected to server")
        print(verification)
    except rospy.ServiceException as e:
        print("fake pi connect service call failed")
    


if __name__ == '__main__':
    rospy.init_node('fake_pi_pkg')
    run_fake_client()