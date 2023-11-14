#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from nav_msgs.msg import Path

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def pose_cb(msg):
    global current_pose
    current_pose = msg

if __name__ == "__main__":
    rospy.init_node("position_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=pose_cb)
    path_pub = rospy.Publisher("drone_path", Path, queue_size=10)  # New path publisher

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while (not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 3
    pose.pose.position.y = 3
    pose.pose.position.z = 2

    # Send a few setpoints before starting
    for i in range(100):
        if (rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    path_msg = Path()
    path_msg.header.frame_id = "map"  # Adjust the frame_id to match your world frame

    while (not rospy.is_shutdown()):
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if set_mode_client.call(offb_set_mode).mode_sent:
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if arming_client.call(arm_cmd).success:
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()

        local_pos_pub.publish(pose)

        # curr_pos = current_pose.pose.position

        # Add the current drone position to the path
        point = PoseStamped()
        point.pose.position.x = current_pose.pose.position.x
        point.pose.position.y = current_pose.pose.position.y
        point.pose.position.z = current_pose.pose.position.z
        path_msg.poses.append(point)

        # Publish the path
        # path_msg.header.stamp = rospy.Time.now()
        path_pub.publish(path_msg)

        rate.sleep()