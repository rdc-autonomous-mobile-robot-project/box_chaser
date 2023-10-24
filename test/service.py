#!/usr/bin/env python3
import rospy
from std_srvs.srv import SetBool, SetBoolResponse

def callback_srv(data):
    resp = SetBoolResponse()
    if data.data == True:
        resp.message = "called"
        resp.success = True
    else:
        resp.message = "ready"
        resp.success = False
    print(resp.message)
    return resp

if __name__ == "__main__":
    rospy.init_node("srv_server")
    srv = rospy.Service('service_call', SetBool, callback_srv)
    print("ready")
    rospy.spin()