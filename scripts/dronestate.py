#!/usr/bin/python3

import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped, Vector3Stamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from numpy import NaN

current_state = State()
current_pose = PoseStamped()
current_velocity = TwistStamped()

def state_cb(msg):
    global current_state
    current_state = msg

def pose_cb(msg):
    global current_pose
    current_pose = msg

def velocity_cb(msg):
    global current_velocity
    current_velocity = msg
    print("Velocity callback called")


if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=pose_cb)
    velocity_sub = rospy.Subscriber("mavros/local_position/velocity", TwistStamped, callback=velocity_cb)


    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    # pose = PoseStamped()

    # pose.pose.position.x = NaN
    # pose.pose.position.y = NaN
    # pose.pose.position.z = NaN

    vel = TwistStamped()

    vel.twist.linear.x = 1.0
    vel.twist.linear.y = 0.0
    vel.twist.linear.z = 0.5

    vel.twist.angular.x = 0.0
    vel.twist.angular.y = 0.0
    vel.twist.angular.z = 0.5

    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        local_vel_pub.publish(vel)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            
                last_req = rospy.Time.now()

         # Get position and orientation of the drone
        pos = current_pose.pose.position
        ori = current_pose.pose.orientation
        rospy.loginfo("Drone position: x=%.2f, y=%.2f, z=%.2f" % (pos.x, pos.y, pos.z))
        rospy.loginfo("Drone orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f" % (ori.x, ori.y, ori.z, ori.w))
        # rospy.loginfo("Drone velocity: x=%.2f, y=%.2f, z=%.2f" % (current_velocity.vector.x, current_velocity.vector.y, current_velocity.vector.z))
        # rospy.loginfo("Drone velocity: vx=%.2f, vy=%.2f, vz=%.2f" % (current_velocity.twist.linear.x, current_velocity.twist.linear.y, current_velocity.twist.linear.z))
    
        local_vel_pub.publish(vel)

        rate.sleep()
