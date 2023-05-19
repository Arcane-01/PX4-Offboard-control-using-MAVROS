#!/usr/bin/python3

# Update: This code attempts to implement the position control in z direction and calculating thrust for the same.
"""
The 2D quadcoptor model can be modelled as:

z'' = -g + (u1/m) * cos(phi)

y'' = -(u1/m) * sin(phi)

phi'' = u2 * 1/ I_xx
"""
import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import State, AttitudeTarget, PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import tf.transformations as tf
from math import pi

current_state = State()
current_pose = PoseStamped()
current_velocity = TwistStamped()

"""
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters
"""

class QuadrotorController: # class for Quadrotor Control

    def __init__(self):

        # attributes for y
        self.y_setpoint = 0.0
        self.y_kp = 0.0
        self.y_kd = 0.0
        self.y_ki = 0.0
        self.y_error_sum = 0.0
        self.y_prev_error = 0.0
        self.y_error_integral = 0.0
        self.y_error_derivative = 0.0

        # attributes for z
        self.z_setpoint = 6.0
        self.z_kp = 0.25
        self.z_kd = 10.0
        self.z_ki = 0.008
        self.z_error_sum = 0.0
        self.z_prev_error = 0.0
        self.z_error_integral = 0.0
        self.z_error_derivative = 0.0

        # attributes for roll
        self.roll_setpoint =  0.0
        self.roll_kp = 5.0
        self.roll_ki = 0.0
        self.roll_kd = 1.0
        self.roll_error_sum = 0.0
        self.roll_prev_error = 0.0
        self.roll_error_integral = 0.0
        self.roll_error_derivative = 0.0
    
    # Get the values of the position of the quadrotor
    def get_position(self, current_pose):
        # Get the x, y, z positions from the current_pose
        pos = current_pose.pose.position
        return(pos.x, pos.y, pos.z)
    
    # Get current roll angle from the orientation quaternion
    def get_roll(self, current_pose):
        quaternion = (
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w
        )
        roll = tf.euler_from_quaternion(quaternion)[0]
        return roll
    
    def compute_z_control(self):
        global current_pose
        __, __, z_position = self.get_position(current_pose)
        rospy.loginfo("Pose.z = %.4f" % z_position)

        z_error = self.z_setpoint - z_position

        # Compute the integral and derivative terms
        self.z_error_integral += z_error
        self.z_error_derivative = (z_error - self.z_prev_error)

        # Update the previous error term
        self.z_prev_error = z_error

        # Compute the control output
        z_command = (self.z_kp * z_error) + (self.z_ki * self.z_error_integral) + (self.z_kd * self.z_error_derivative)

        return z_command

    def compute_y_control(self):
        global current_pose
        __, y_position, __ = self.get_position(current_pose)
        rospy.loginfo("Pose.y = %.4" %(y_position))

        y_error = self.y_setpoint - y_position

        # Compute the derivative and integral terms
        self.y_error_integral += y_error
        self.y_error_derivative = (y_error - self.y_prev_error)

        # Update the value of the previous y_error term
        self.y_prev_error = y_error

        # Compute the control output
        y_command = (self.y_kp * y_error) + (self.y_ki * self.y_error_integral) + (self.y_kd * self.y_error_derivative)

        return y_command

    def compute_roll_control(self):
        global current_pose

        # Get the value of roll of the quadrotor 
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

## callback functions to update the current_state, current_pose and current_velocity variables

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
        z_command = controller.compute_z_control()
        # initial attitude target for enabling the offboard mode
        attitude_target = AttitudeTarget()
        attitude_target.header.stamp = rospy.Time.now()
        attitude_target.type_mask = (
            AttitudeTarget.IGNORE_ATTITUDE
            )
        attitude_target.body_rate.x = roll_command
        attitude_target.body_rate.y = 0.0
        attitude_target.body_rate.z = 0.0
        attitude_target.thrust = z_command
        attitude_command_pub.publish(attitude_target)
        
        rate.sleep()