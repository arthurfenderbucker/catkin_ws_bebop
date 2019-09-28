#!/usr/bin/env python
'''
Code made for test commum state massages and validade the capability
to cacht battery porcentage
'''

import rospy
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged
#import CommonCommonStateBatteryStateChanged.msg
import time
print('antes')
rospy.set_param('~states/enable_commonstate_batterystatechanged', 1)
print('depois')
battery = CommonCommonStateBatteryStateChanged()
print('bat exist')

def bat_state (bat):
    print ('entrou no batstate')
    global battery
    rospy.loginfo("%s \n", bat.percent)
    battery.percent = bat.percent
    print (battery.percent)

rospy.init_node('Bebop_2_Battery_Porcentage')
print('node iniciado')
rate = rospy.Rate(1)
print('rate definido')
rospy.Subscriber('/bebop/states/common/CommonState/BatteryStateChanged', CommonCommonStateBatteryStateChanged, bat_state)
print('subfeito')
i = 0

while i < 24:
    print(" -------- Battery level --------- ")
    print (battery.percent)
    rate.sleep()
    try:
        rate.spin()
    except Exception as e:
        pass
    i+=1
