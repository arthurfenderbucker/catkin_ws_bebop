#!/usr/bin/env python
import rospy
import numpy as np

from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

rospy.init_node('service_test')


def use_uppercase(request):
    assert isinstance(request, SetBoolRequest)
    message = "Oi tudo bom"
    if request.data:
        message = message.upper()
    else:
        message = message.lower()
    print("haha", message)
    return SetBoolResponse(True, "Boolean set. New message value: {}".format(message))
s= rospy.Service("/test/use_up", SetBool,use_uppercase)
rospy.spin()