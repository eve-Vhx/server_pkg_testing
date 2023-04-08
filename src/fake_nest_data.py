#!/usr/bin/env python3
import rospy
import random
from msg_pkg.msg import NestChargeFeedback
from msg_pkg.msg import NestBeaconFeedback
from msg_pkg.msg import fakeChargeMsg, fakeBeaconMsg

def run_fake_nest_data():
    while True:
        fake_charge_msg = fakeChargeMsg()
        fake_beacon_msg = fakeBeaconMsg()
        fake_charge_msg.charging = True
        fake_beacon_msg.beacon_on = True

        fake_charge_pub.publish(fake_charge_msg)
        fake_beacon_pub.publish(fake_beacon_msg)

        rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('fake_nest_data')
    fake_charge_pub = rospy.Publisher('NEST11012/Charge_cntl/feedback', fakeChargeMsg, queue_size=10)
    fake_beacon_pub = rospy.Publisher('NEST11012/Beacon_cntl/feedback', fakeBeaconMsg, queue_size=10)

    run_fake_nest_data()
    rospy.spin()