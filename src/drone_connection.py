#!/usr/bin/env python3
import rospy
from msg_pkg.msg import connections_drone
from msg_pkg.msg import feedbackMsg
from msg_pkg.msg import telemMsg
from msg_pkg.srv import masterConnect, masterConnectResponse

from mavros_msgs.msg import GPSRAW
from mavros_msgs.msg import State
from sensor_msgs.msg import Range
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float64

import threading


class DroneConnection:
    def __init__(self, id):
        self.timer = threading.Timer(5,self.timeout)
        self.drone_connect_service = rospy.Service(id + 'drone_telem_connect', masterConnect, self.run_routine)

    def run_routine(self,req):
        self.id = req.id
        rospy.Subscriber(self.id + "/connection_checks", connections_drone, self.connections_cb)
        self.px4 = False
        self.mavros = False
        self.connected = False
        self.drone_connecter_feedback = rospy.Publisher(self.id + "/drone_connecter_feedback", feedbackMsg, queue_size=10)
        self.drone_telem_pub = rospy.Publisher(req.id + "/ui_telem_data", telemMsg, queue_size=10)

        #MAVROS telemetry data
        self.mavros_telem_gps = {
            "lat" : 30.26868,
            "lon" : -97.74058,
            "alt" : 0
        }
        self.mavros_telem_state = {
            "state" : " ",
            "armed" : False
        }
        self.mavros_telem_dystancez = {
            "dist_z" : 0
        }
        self.mavros_telem_vel = {
            "vel_x" : 0,
            "vel_y" : 0,
            "vel_z" : 0
        }
        self.mavros_telem_battery = {
            "battery_percent" : 0
        }

        self.mavros_telem_compass = {
            "hdg" : 0
        }
        rospy.Subscriber(self.id + "/mavros/gpsstatus/gps1/raw", GPSRAW, self.mavros_gps_cb)
        rospy.Subscriber(self.id + "/mavros/state", State, self.mavros_state_cb)
        rospy.Subscriber(self.id + "/mavros/distance_sensor/hrlv_ez4_pub", Range, self.mavros_distancez_cb)
        rospy.Subscriber(self.id + "/mavros/setpoint_velocity/cmd_vel", TwistStamped, self.mavros_vel_cb)
        rospy.Subscriber(self.id + "/mavros/battery", BatteryState, self.mavros_battery_cb)
        rospy.Subscriber(self.id + "/mavros/global_position/compass_hdg", Float64, self.mavros_compass_cb)

        self.timer.start()

        self.publish_telem_data()

        return masterConnectResponse(True)

    def timeout(self):
        self.connected = False
        self.mavros = False
        self.px4 = False
        

    def connections_cb(self,msg):
        self.connected = True
        self.timer.cancel()
        self.timer = threading.Timer(5,self.timeout)
        self.timer.start()
        if (msg.mavros == True):
            self.mavros = True
        elif (msg.mavros == False):
            self.mavros = False
        if (msg.px4 == True):
            self.px4 = True
        elif (msg.px4 == False):
            self.px4 = False

    def mavros_gps_cb(self,data):
        self.mavros_telem_gps = {
            "lat" : data.lat*10**(-7),
            "lon" : data.lon*10**(-7),
            "alt" : data.alt*10**(-3),
        }
    
    def mavros_state_cb(self,data):
        self.mavros_telem_state = {
            "state" : data.mode,
            "armed" : data.armed
        }

    def mavros_distancez_cb(self,data):
        self.mavros_telem_dystancez = {
            "dist_z" : data.range
        }

    def mavros_vel_cb(self,data):
        self.mavros_telem_vel = {
            "vel_x" : data.twist.linear.x,
            "vel_y" : data.twist.linear.y,
            "vel_z" : data.twist.linear.z
        }

    def mavros_battery_cb(self,data):
        self.mavros_telem_battery = {
            "battery_percent" : data.percentage
        }

    def mavros_compass_cb(self,data):
        self.mavros_telem_compass = {
            "hdg" : data.data
        }

    def publish_telem_data(self):
        while(not rospy.is_shutdown()):
            ui_telem_msg = telemMsg()
            ui_telem_msg.lat = self.mavros_telem_gps["lat"]
            ui_telem_msg.lon = self.mavros_telem_gps["lon"]
            ui_telem_msg.alt = self.mavros_telem_gps["alt"]
            ui_telem_msg.state = self.mavros_telem_state["state"]
            ui_telem_msg.armed = self.mavros_telem_state["armed"]
            ui_telem_msg.dist_z = self.mavros_telem_dystancez["dist_z"]
            ui_telem_msg.vel_x = self.mavros_telem_vel["vel_x"]
            ui_telem_msg.vel_y = self.mavros_telem_vel["vel_y"]
            ui_telem_msg.vel_z = self.mavros_telem_vel["vel_z"]
            ui_telem_msg.battery = self.mavros_telem_battery["battery_percent"]
            ui_telem_msg.compass = self.mavros_telem_compass["hdg"]
            ui_telem_msg.connected = self.connected
            ui_telem_msg.mavros = self.mavros
            ui_telem_msg.px4 = self.px4

            self.drone_telem_pub.publish(ui_telem_msg)
            rospy.sleep(0.5)


            
