#!/usr/bin/env python
# Andrew Torgesen 2016
import rospy
from fcu_common.msg import FW_Waypoint

def pub_waypoints():
    pub = rospy.Publisher('waypoint_path', FW_Waypoint, queue_size=10)
    rospy.init_node('ros_plane_path_planner')
    rate = rospy.Rate(2) # 2 hz
    Va = 30.0
    wps = [
        400, 0, -100, 0, Va,
        523.607, -19.5774, -100, 0, Va,
        635.114, -76.3932, -100, 0, Va,
        723.607, -164.886, -100, 0, Va,
        780.423, -276.393, -100, 0, Va,
        800, -400, -100, 0, Va,
        780.423, -523.607, -100, 0, Va,
        723.607, -635.114, -100, 0, Va,
        635.114, -723.607, -100, 0, Va,
        523.607, -780.423, -100, 0, Va,
        400, -800, -100, 0, Va,
        276.393, -780.423, -100, 0, Va,
        164.886, -723.607, -100, 0, Va,
        76.3932, -635.114, -100, 0, Va,
        19.5774, -523.607, -100, 0, Va,
        0, -400, -100, 0, Va,
        19.5774, -276.393, -100, 0, Va,
        76.3932, -164.886, -100, 0, Va,
        164.886, -76.3932, -100, 0, Va,
        276.393, -19.5774, -100, 0, Va
    ]
    #print(len(wps)/5)
    i = 0
    #for i in range(0,len(wps)/5):
    while not rospy.is_shutdown() and i < len(wps)/5:
        #print('hey')#-----------------------------------------------
        new_waypoint = FW_Waypoint()
        new_waypoint.w[0] = wps[i*5 + 0]
        new_waypoint.w[1] = wps[i*5 + 1]
        new_waypoint.w[2] = wps[i*5 + 2]
        new_waypoint.chi_d = wps[i*5 + 3]
        new_waypoint.chi_valid = False
        new_waypoint.Va_d = wps[i*5 + 4]

        pub.publish(new_waypoint)
        #print('hey2')#-----------------------------------------------
        i += 1
        rate.sleep()

if __name__ == '__main__':
    #gtk.gdk.threads_init()
    try:
        pub_waypoints()
    except rospy.ROSInterruptException:
        pass
