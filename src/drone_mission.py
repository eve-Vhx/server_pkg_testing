#!/usr/bin/env python3
import rospy
import actionlib
from msg_pkg.msg import server_px4_reqGoal, server_px4_reqAction, server_px4_reqResult, server_px4_reqFeedback
from msg_pkg.srv import UiReq, UiReqResponse
from msg_pkg.msg import telemMsg

class DroneMission:
    def __init__(self):
        self.mavros = False
        self.px4 = False
        self.connected = False
        self.drone_mission_service = rospy.Service('ui_mission_req',UiReq, self.run_routine)
        self.success = False
    
    def run_routine(self,req):
        print("running mission request")
        self.id = req.drone_id
        self.mission_goal = [req.lat, req.lon, req.alt]
        self.connected_server = False
        self.mission_action_client = actionlib.SimpleActionClient(self.id + '/mavros/smr_px4_command/d1_cmd_action', server_px4_reqAction)
        rospy.Subscriber(self.id + "/ui_telem_data", telemMsg, self.connections_cb)

        self.check_send_mission()

        UiReqResponse(self.success)

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
        mission_goal_msg = server_px4_reqGoal(lat=self.mission_goal[0], lon=self.mission_goal[1], alt=self.mission_goal[2], yaw_rad=0, timestamp=rospy.Time.now().secs, mission_type=2, cruise_alt=10)
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
    rospy.init_node('server_mission_node')
    DroneMission()
    rospy.spin()