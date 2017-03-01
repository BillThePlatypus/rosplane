#include <ros/ros.h>
#include <fcu_common/FW_Waypoint.h>

#define num_waypoints 20

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_plane_path_planner");

    ros::NodeHandle nh_;
    ros::Publisher waypointPublisher = nh_.advertise<fcu_common::FW_Waypoint>("waypoint_path",10);

    ros::Duration(0.5).sleep();

    float Va = 30;//11;
    float wps[5*num_waypoints] = {
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
               };

    for(int i(0);i<num_waypoints;i++)
    {

        fcu_common::FW_Waypoint new_waypoint;

        new_waypoint.w[0] = wps[i*5 + 0];
        new_waypoint.w[1] = wps[i*5 + 1];
        new_waypoint.w[2] = wps[i*5 + 2];
        new_waypoint.chi_d = wps[i*5 + 3];

        new_waypoint.chi_valid = false;//true;
        new_waypoint.Va_d = wps[i*5 + 4];

        waypointPublisher.publish(new_waypoint);

        ros::Duration(0.5).sleep();
    }

    return 0;
}
