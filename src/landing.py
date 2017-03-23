# created by Dane
import math
import rospy
from fcu_common.msg import FW_waypoint

pub_waypoint = rospy.Publisher('waypoint_path', FW_waypoint, queue_size=300)

def create_landing_waypoints(height, turn_radius, num_waypoints, distance_between_points, initial_velocity):
	down = num_waypoints*[0];
	north = num_waypoints*[0];
	east = num_waypoints*[0];
	angle = num_waypoints*[0];
	velocity = num_waypoints*[0];
	waypoints = num_waypoints*[[]];

	east_offset = turn_radius*2;
	h_ratio = float(height)/num_waypoints;
	start_turn = num_waypoints/2 - turn_radius/distance_between_points;
	end_turn = num_waypoints/2 + turn_radius/distance_between_points;
	turn_points = end_turn - start_turn;

	for i in range(0, num_waypoints):
		down[i] =  height - float(i)*h_ratio;
		if i <= start_turn:
			east[i] = east_offset;
			north[i] = i*distance_between_points;
		elif i > start_turn and i < end_turn:
            #logic half-circle
			east[i] = turn_radius + turn_radius*math.sin(math.pi/2 - math.pi*float(i-start_turn)/(turn_points));
			north[i] = start_turn*distance_between_points + turn_radius*math.cos(math.pi/2-math.pi*float(i-start_turn)/(turn_points));
		else:
			east[i] = 0;
			north[i] = north[i-1]-distance_between_points;
        #calculate velocity
		velocity[i] = initial_velocity-(float(i)/num_waypoints*initial_velocity);
		waypoints[i]= [north[i], east[i], down[i], angle[i], velocity[i]];
		pub_waypoint.publish(waypoints[i]);
	#return [north, east, down, angle, velocity];
	return waypoints;


waypoints = 300;
turn_radius = 50;
distance_between_points = 10;
height_start = -100;
initial_speed = 30;

waypoint_array = create_landing_waypoints(height_start, turn_radius, waypoints, distance_between_points, initial_speed);

# publish to [-9999,-9999,-9999,0,0] to reset

#print(waypoint_array);


