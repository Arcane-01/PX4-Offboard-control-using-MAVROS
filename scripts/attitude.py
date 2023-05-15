#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import tf.transformations as tf
from math import pi 

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    local_att_pub = rospy.Publisher("mavros/setpoint_attitude/attitude", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # Set Euler angles for orientation
    roll = 0.0  # radians
    pitch = 0.0  # radians
    yaw =  pi/4 # radians
    quaternion = tf.quaternion_from_euler(roll, pitch, yaw)
    
    # To publish the attitude setpoint, a new pose is created with only the orientation set. 
    # The position values should be set to 0 to avoid unexpected behavior in the vehicle's position.
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]

    # Create a new pose for attitude setpoint
    att_pose = PoseStamped()
    att_pose.pose.orientation.x = quaternion[0]
    att_pose.pose.orientation.y = quaternion[1]
    att_pose.pose.orientation.z = quaternion[2]
    att_pose.pose.orientation.w = quaternion[3]

    # Set position values to 0 to avoid conflict with position setpoint
    att_pose.pose.position.x = 0
    att_pose.pose.position.y = 0
    att_pose.pose.position.z = 0

    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        local_att_pub.publish(att_pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent):
                rospy.loginfo("Offboard enabled")
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success):
                    rospy.loginfo("Vehicle armed")
                last_req = rospy.Time.now()

        local_pos_pub.publish(pose)
        local_att_pub.publish(att_pose)

        rate.sleep()
