#!/usr/bin/python3

# Code to make the UAV follow a set of waypoints with the low-level controller of PX4

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from nav_msgs.msg import Odometry, Path

current_state = State()
current_pose = PoseStamped()

global offboard_detect # Flag to detect if OFFBOARD mode has been enabled and the drone has flied
global arm_detect # Flag to check if the drone has been armed once
offboard_detect = False
arm_detect = False

# tolerance value used for switching to the next waypoint
position_tolerance = 0.1

# List of waypoints for the drone to follow
waypoints = [
    [1, 1, 1.5],    # Waypoint 1: x=0, y=0, z=6
    [1, -1, 1.5],   # Waypoint 2: x=0, y=4, z=6
    [-1, -1, 1.5],  # Waypoint 3: x=0, y=4, z=5
    [-1, 1, 1.5]    # Waypoint 4: x=0, y=4, z=4
]

current_waypoint_index = 0

def state_cb(msg):
    global current_state
    current_state = msg

def pose_cb(msg): # subscribed to the mavros/local_position/pose topic
    global current_pose
    current_pose = msg

if __name__ == "__main__":
    rospy.init_node("position_py")

    
    # Subscribers for state, position and velocity
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb, queue_size=1)
    pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=pose_cb, queue_size=1) # used for finding the current roll of the quadrotor 

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    # Create an Odometry publisher
    odom_pub = rospy.Publisher("drone_odom", Odometry, queue_size=10)
    # Create a Path publisher
    path_pub = rospy.Publisher("drone_path", Path, queue_size=10)
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    # initail pose inputs for enabling offboard mode
    pose_i = PoseStamped()

    pose_i.pose.position.x = 0
    pose_i.pose.position.y = 0
    pose_i.pose.position.z = 0

    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose_i)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and current_state.mode != "AUTO.LAND" and (not offboard_detect) and (rospy.Time.now() - last_req) > rospy.Duration(10.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                offboard_detect = True
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and current_state.mode != "AUTO.LAND" and (not arm_detect) and (rospy.Time.now() - last_req) > rospy.Duration(10.0)):
                if(arming_client.call(arm_cmd).success == True):
                    arm_detect = True
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()

        if(current_state.mode == "AUTO.LAND" and (rospy.Time.now() - last_req) > rospy.Duration(10.0)):
            rospy.loginfo("LAND enabled")
            last_req = rospy.Time.now()

        # Current position of the Quadcopter used for switching to the next waypoint
        curr_pos = current_pose.pose.position

        # Check if reached current waypoint
        current_waypoint = waypoints[current_waypoint_index]
        if (abs(curr_pos.x - current_waypoint[0]) < position_tolerance and abs(curr_pos.y - current_waypoint[1]) < position_tolerance and abs(curr_pos.z - current_waypoint[2]) < position_tolerance):
            # Increment waypoint index
            current_waypoint_index += 1
            if current_waypoint_index >= len(waypoints):
                # current_waypoint_index = len(waypoints)-1
                current_waypoint_index = 0
            # Print the current waypoint index if it's not the final waypoint
            if current_waypoint_index < (len(waypoints)-1):
                rospy.loginfo("Switching to waypoint index: {}".format(current_waypoint_index))

        target_waypoint = waypoints[current_waypoint_index]
        # set_pose(local_pos_pub, target_waypoint[0], target_waypoint[1], target_waypoint[2])
        target_pose = PoseStamped()

        target_pose.pose.position.x = target_waypoint[0]
        target_pose.pose.position.y = target_waypoint[1]
        target_pose.pose.position.z = target_waypoint[2]
        
        local_pos_pub.publish(target_pose)

        # Publish the drone's pose as part of the Path message
        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = curr_pos.x
        pose_stamped.pose.position.y = curr_pos.y
        pose_stamped.pose.position.z = curr_pos.z
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "base_link"
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"  # Set the frame ID according to your setup
        path_msg.poses.append(pose_stamped)

        path_pub.publish(path_msg)

        rate.sleep()