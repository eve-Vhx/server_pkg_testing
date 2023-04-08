#!/usr/bin/env python3
import rospy
import actionlib
from msg_pkg.msg import NestChargeAction, NestChargeGoal, NestChargeFeedback
from msg_pkg.srv import nestChargeUi, nestChargeUiResponse

class NestCharge:
    def __init__(self):
        self.nest_charge_service = rospy.Service('ui_nest_charge_req', nestChargeUi, self.handle_charge_cb)
        self.success = False

    def handle_charge_cb(self,req):
        self.charge_action_client = actionlib.SimpleActionClient(req.id + '/Charge_cntl', NestChargeAction)
        if(self.charge_action_client.wait_for_server(timeout=rospy.Duration(5.0))):
            self.charge_goal = NestChargeGoal(charge_drone=req.charge)
            self.charge_action_client.send_goal(self.charge_goal)
            self.success = True
            print("Sending the charge request through")
        else:
            self.success = False
            print("Cannot send charge request. Timeout")
            
        
        return nestChargeUiResponse(self.success)

if __name__ == '__main__':
    rospy.init_node('nest_charge_node')
    NestCharge()
    rospy.spin()
