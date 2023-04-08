#!/usr/bin/env python3
import rospy
from msg_pkg.srv import masterConnect, masterConnectResponse
from msg_pkg.msg import uiMasterList
from msg_pkg.srv import UiReq

from nest_connection import NestConnection
from drone_connection import DroneConnection


class ServerConnecter:
    def __init__(self,name):
        self.drone_ui_array = []
        self.nest_ui_array = []
        self.drone_pi_connect_service = rospy.Service('drone_pi_connect_master', masterConnect, self.handle_drone_connect_cb)
        self.nest_pi_connect_service = rospy.Service('nest_pi_connect_master', masterConnect, self.handle_nest_connect_cb)
        self.ui_drone_pub = rospy.Publisher('drone_master_list', uiMasterList, queue_size=10)
        self.ui_nest_pub = rospy.Publisher('nest_master_list', uiMasterList, queue_size=10)
        print("Started server connect service")
        self.run_routine()

    def run_routine(self):
        while True:
            drone_master_msg = uiMasterList()
            drone_master_msg.ui_master_list = self.drone_ui_array
            self.ui_drone_pub.publish(drone_master_msg)

            nest_master_msg = uiMasterList()
            nest_master_msg.ui_master_list = self.nest_ui_array
            self.ui_nest_pub.publish(nest_master_msg)

            rospy.sleep(3)
    
    def handle_drone_connect_cb(self,req):

        drone_exists = False
        for droneid in self.drone_ui_array:
            if(droneid == req.id):
                drone_exists = True
        
        if(not drone_exists):
            print("Drone does not exist")
            self.drone_ui_array.append(req.id)
            drone_connection_obj = DroneConnection(req.id)
            rospy.wait_for_service(req.id + 'drone_telem_connect')
            try:
                drone_telem_client = rospy.ServiceProxy(req.id + 'drone_telem_connect', masterConnect)
                print(drone_telem_client(req.id))
            except rospy.ServiceException as e:
                print("Cannot setup new drone for telemetry")
        else:
            print("Drone already exists")

        return masterConnectResponse(True)

    def handle_nest_connect_cb(self,req):
        print("nest connected to server")
        nest_exists = False
        for nestid in self.nest_ui_array:
            if(nestid == req.id):
                nest_exists = True

        if(not nest_exists):
            self.nest_ui_array.append(req.id)
            nest_connection_obj = NestConnection(req.id)
            rospy.wait_for_service(req.id + 'nest_telem_connect')
            try:
                nest_telem_client = rospy.ServiceProxy(req.id + 'nest_telem_connect', masterConnect)
                print(nest_telem_client(req.id))
            except rospy.ServiceException as e:
                print("Cannot setup new nest for telemetry")

        return masterConnectResponse(True)




if __name__ == '__main__':
    rospy.init_node('server_connecter', anonymous=True)
    ServerConnecter(rospy.get_name())
    rospy.spin()



