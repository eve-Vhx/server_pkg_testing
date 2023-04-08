#!/usr/bin/env python3
import rospy
import random
from mavros_msgs.msg import GPSRAW
from mavros_msgs.msg import State
from sensor_msgs.msg import Range
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import BatteryState

def run_fake_mavros():
    while True:
        fake_gps_msg = GPSRAW()
        fake_state_msg = State()
        fake_dist_msg = Range()
        fake_vel_msg = TwistStamped()
        fake_batt_msg = BatteryState()

        fake_gps_msg.lat = random.randint(303910000,303920000)
        fake_gps_msg.lon = random.randint(-977270000,-977170000)
        fake_gps_msg.alt = random.randint(240000,241000)
        fake_state_msg.mode = 'POSCTL'
        fake_state_msg.armed = False
        fake_dist_msg.range = random.uniform(5,7)

        fake_gps_pub.publish(fake_gps_msg)
        fake_state_pub.publish(fake_state_msg)
        fake_distance_pub.publish(fake_dist_msg)

        rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('fake_mavros')
    fake_gps_pub = rospy.Publisher('QROW11021/mavros/gpsstatus/gps1/raw', GPSRAW, queue_size=10)
    fake_state_pub = rospy.Publisher('QROW11021/mavros/state', State, queue_size=10)
    fake_distance_pub = rospy.Publisher('QROW11021/mavros/distance_sensor/hrlv_ez4_pub', Range, queue_size=10)
    fake_vel_pub = rospy.Publisher('QROW11021/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    fake_battery_pub = rospy.Publisher('QROW11021/mavros/battery', BatteryState, queue_size=10)

    run_fake_mavros()
    rospy.spin()