#!/usr/bin/env python3
import rospy
import actionlib
from msg_pkg.msg import server_px4_reqGoal, server_px4_reqAction, server_px4_reqResult, server_px4_reqFeedback
from msg_pkg.msg import server_px4_req_waypointsGoal, server_px4_req_waypointsAction, server_px4_req_waypointsResult, server_px4_req_waypointsFeedback
from msg_pkg.srv import UiWaypointReq, UiWaypointReqResponse
from msg_pkg.msg import telemMsg

class DroneMissionWaypoints:
    def __init__(self):
        self.mavros = False
        self.px4 = False
        self.connected = False
        self.drone_mission_service = rospy.Service('ui_waypoint_mission_req',UiWaypointReq, self.run_routine)
        self.success = False

        self.waypoint_mission_goal = {
            'lats': [],
            'lons': [],
            'alts': [],
            'dest_lat': 30.38776,
            'dest_lon': -97.72837,
            'dest_alt': 260
        }
    
    def run_routine(self,req):
        print("running waypoint mission request")
        self.id = req.drone_id
        self.waypoint_mission_goal["lats"] = req.lat
        self.waypoint_mission_goal["lons"] = req.lon
        self.waypoint_mission_goal["alts"] = req.alt
        self.waypoint_mission_goal["dest_lat"] = req.dest_lat
        self.waypoint_mission_goal["dest_lon"] = req.dest_lon
        self.waypoint_mission_goal["dest_alt"] = req.dest_alt
        self.connected_server = False
        self.mission_action_client = actionlib.SimpleActionClient(self.id + '/mavros/smr_px4_command/d1_cmd_action_waypoints', server_px4_req_waypointsAction)
        rospy.Subscriber(self.id + "/ui_telem_data", telemMsg, self.connections_cb)

        self.check_send_mission()

        UiWaypointReqResponse(self.success)

    def connections_cb(self,msg):
        self.px4 = msg.px4
        self.mavros = msg.mavros
        self.connected = msg.connected

        print("PX4: ")
        print(self.px4)
        print("MAVROS: ")
        print(self.mavros)
        print("CONNECTED: ")
        print(self.connected)

    def initiate_connection(self):
        return self.mission_action_client.wait_for_server(timeout=rospy.Duration(5.0))

    def send_goal_pi(self):
        mission_goal_msg = server_px4_req_waypointsGoal(lat=self.waypoint_mission_goal["lats"], 
                                                        lon=self.waypoint_mission_goal["lons"], 
                                                        alt=self.waypoint_mission_goal["alts"], 
                                                        dest_lat=self.waypoint_mission_goal["dest_lat"],
                                                        dest_lon=self.waypoint_mission_goal["dest_lon"],
                                                        dest_alt=self.waypoint_mission_goal["dest_alt"],
                                                        yaw_rad=0, 
                                                        timestamp=rospy.Time.now().secs, 
                                                        mission_type=4, 
                                                        cruise_alt=self.waypoint_mission_goal["dest_alt"])
        self.mission_action_client.send_goal(mission_goal_msg)
        self.mission_action_client.wait_for_result()
        print(self.mission_action_client.get_result())
        self.success = True

    def check_send_mission(self):
        if(self.initiate_connection() == True):
            self.connected_server = True
            self.send_goal_pi()
        else:
            self.connected_server = False
            self.success = False
            print("The action client not connected to the pi server")
    
                

if __name__ == '__main__':
    rospy.init_node('server_waypoint_mission_node')
    DroneMissionWaypoints()
    rospy.spin()