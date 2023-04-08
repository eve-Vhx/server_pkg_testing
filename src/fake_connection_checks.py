#!/usr/bin/env python3
import rospy
from msg_pkg.msg import connections_drone

def run_fake_connections():
    while True:
        fake_conn_msg = connections_drone()
        fake_conn_msg.mavros = True
        fake_conn_msg.px4 = True
        fake_conn_msg.ros_timestamp = rospy.Time.now()

        fake_conn_pub.publish(fake_conn_msg)
        rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('fake_connection_checks')
    fake_conn_pub = rospy.Publisher('QROW11021/connection_checks', connections_drone, queue_size=10)
    run_fake_connections()
    rospy.spin()
