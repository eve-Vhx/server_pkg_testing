#!/usr/bin/env python3
import rospy
import actionlib
from msg_pkg.msg import NestBeaconAction, NestBeaconGoal, NestBeaconFeedback
from msg_pkg.srv import nestBeaconUi, nestBeaconUiResponse

class NestBeacon:
    def __init__(self):
        self.nest_beacon_service = rospy.Service('ui_nest_beacon_req', nestBeaconUi, self.handle_beacon_cb)
        self.success = False

    def handle_beacon_cb(self,req):
        self.beacon_action_client = actionlib.SimpleActionClient(req.id + '/Beacon_cntl', NestBeaconAction)
        if(self.beacon_action_client.wait_for_server(timeout=rospy.Duration(5.0))):
            self.beacon_goal = NestBeaconGoal(beacon_state=req.beacon_on)
            self.beacon_action_client.send_goal(self.beacon_goal)
            self.success = True
            print("Sending the beacon request through")
        else:
            self.success = False
            print("Cannot send beacon request. Timeout")
            
        
        return nestBeaconUiResponse(self.success)

if __name__ == '__main__':
    rospy.init_node('nest_beacon_node')
    NestBeacon()
    rospy.spin()