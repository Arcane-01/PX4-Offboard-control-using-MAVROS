#!/usr/bin/python3

# Update: This is the final working version of the code
import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import State, AttitudeTarget, PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import tf.transformations as tf
from math import pi

current_state = State()
current_pose = PoseStamped()
current_velocity = TwistStamped()

class QuadrotorController: # class for Quadrotor Control

    def __init__(self):

        # attributes for roll
        self.roll_setpoint =  0.0
        self.roll_kp = 5.0
        self.roll_ki = 0.0
        self.roll_kd = 1.0
        self.roll_error_sum = 0.0
        self.roll_prev_error = 0.0
        self.roll_error_integral = 0.0
        self.roll_error_derivative = 0.0
    
    # Get the value of roll of the quadrotor 
    def get_roll(self, current_pose):
        quaternion = (
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w
        )
        roll = tf.euler_from_quaternion(quaternion)[0]
        return roll
    
    def compute_roll_control(self):
        global current_pose

        # Get current roll angle from the orientation quaternion
        roll = self.get_roll(current_pose)
        rospy.loginfo("Roll  =%.4f" % (roll))
        roll_error = self.roll_setpoint - roll
        # rospy.loginfo("Roll Error =%.2f" % (roll_error))

        # Compute the integral and derivative terms
        self.roll_error_integral += roll_error 
        self.roll_error_derivative = (roll_error - self.roll_prev_error)

        # Update the previous error term
        self.roll_prev_error = roll_error

        # Compute the control output
        roll_command = (self.roll_kp * roll_error) + (self.roll_ki * self.roll_error_integral) + (self.roll_kd * self.roll_error_derivative)

        return roll_command
        # self.publish_roll_command(roll_command)

# callback functions to update the current_state, current_pose and current_velocity variables

def state_cb(msg): # subscribed to mavros/state topic
    global current_state
    current_state = msg

def pose_cb(msg): # subscribed to mavros/local_position/pose topic
    global current_pose
    current_pose = msg

if __name__ == "__main__":
    rospy.init_node("my_controller_py")

    # Subscribers for state, position and velocity
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb, queue_size=1)
    pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=pose_cb, queue_size=1) # used for finding the current roll of the quadrotor 
   
    # Publishers for position and attitude
    attitude_command_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    
    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    while(not rospy.is_shutdown() and not current_state.connected):
        rospy.loginfo('Waiting for FCU connection...') # printed on terminal
        rate.sleep()

    rospy.loginfo('FCU Connected!')    

    controller = QuadrotorController()

    # initial attitude target for enabling the offboard mode
    attitude_target_i = AttitudeTarget()
    attitude_target_i.header.stamp = rospy.Time.now()
    attitude_target_i.type_mask = (
            AttitudeTarget.IGNORE_ATTITUDE
        )
    attitude_target_i.body_rate.x = 0.0
    attitude_target_i.body_rate.y = 0.0
    attitude_target_i.body_rate.z = 0.0
    attitude_target_i.thrust = 0.75


    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        attitude_command_pub.publish(attitude_target_i)
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

        
        roll_command = controller.compute_roll_control()

        # initial attitude target for enabling the offboard mode
        attitude_target = AttitudeTarget()
        attitude_target.header.stamp = rospy.Time.now()
        attitude_target.type_mask = (
            AttitudeTarget.IGNORE_ATTITUDE
            )
        attitude_target.body_rate.x = roll_command
        attitude_target.body_rate.y = 0.0
        attitude_target.body_rate.z = 0.0
        attitude_target.thrust = 0.75
        attitude_command_pub.publish(attitude_target)
        
        rate.sleep()