#!/usr/bin/python3

# Update: This code has a separate class for PID controller and the publish roll_command_function of the QuadrotorController class is empty here
import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import State, AttitudeTarget, PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from numpy import NaN
from sensor_msgs.msg import Imu
import tf.transformations as tf
from math import pi

current_state = State()
current_pose = PoseStamped()
current_velocity = TwistStamped()

# array for plotting the roll error
global roll_error_arr 
roll_error_arr = []

# callback functions to update the current_state, current_pose and current_velocity variables

def state_cb(msg): # subscribed to mavros/state topic
    global current_state
    current_state = msg

def pose_cb(msg): # subscribed to mavros/local_position/pose topic
    global current_pose
    current_pose = msg

def velocity_cb(msg): # subscribed to mavros/local_position/velocity topic
    global current_velocity
    current_velocity = msg
    # print("Velocity callback called") # for debugging

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0.0
        self.integral = 0.0

    def compute_control_output(self, error):
        # Proportional term
        p_term = self.kp * error

        # Integral term
        self.integral += error
        i_term = self.ki * self.integral

        # Derivative term
        derivative = error - self.last_error
        d_term = self.kd * derivative

        # Compute the control output
        control_output = p_term + i_term + d_term

        # Update the last error for the next iteration
        self.last_error = error

        return control_output
    
class QuadrotorController: # class for Quadrotor Control

    def __init__(self):
        rospy.init_node('quadrotor_controller')
        self.position_command_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.imu_sub = rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)

        # the sample time
        self.dt = 0.1

        # attributes for roll
        self.roll_setpoint = 0.0
        self.roll_kp = 0.5
        self.roll_ki = 0.0
        self.roll_kd = 0.1
        self.roll_error_sum = 0.0
        self.roll_prev_error = 0.0
        self.roll_error_integral = 0.0
        self.roll_error_derivative = 0.0
    
    # Get the value of roll of the quadrotor 
    def get_roll(self, current_state):
        # Get current roll angle from the orientation quaternion
        quaternion = (
            current_state.orientation.x,
            current_state.orientation.y,
            current_state.orientation.z,
            current_state.orientation.w
        )
        roll = tf.euler_from_quaternion(quaternion)[0]
        return roll
    
    def compute_roll_control(self):
        global current_state
        global roll_error_arr
    # Get current roll angle from the orientation quaternion
        quaternion = (
            current_state.orientation.x,
            current_state.orientation.y,
            current_state.orientation.z,
            current_state.orientation.w
        )
        roll = tf.euler_from_quaternion(quaternion)[0]

        roll_error = self.roll_setpoint - roll

        # appending the error to the error array
        roll_error_arr.append(roll_error)

        # Compute the integral and derivative terms
        self.roll_error_integral += roll_error 
        self.roll_error_derivative = (roll_error - self.roll_prev_error)

        # Update the previous error term
        self.roll_prev_error = roll_error

        # Compute the control output
        roll_command = (self.roll_kp * roll_error) + (self.roll_ki * self.roll_error_integral) + (self.roll_kd * self.roll_error_derivative)

        self.publish_roll_command(roll_command)

    def publish_roll_command(self,roll_command):
        pass

        # roll_out = (self.roll_kp * ) + (self.roll_ki*)+ (self.roll_kd*)
    def run(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("my_controller_py")

    # Subscribers for state, position and velocity
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=pose_cb)
    velocity_sub = rospy.Subscriber("mavros/local_position/velocity", TwistStamped, callback=velocity_cb)

    # Publishers for position and velocity
    
    # local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    # local_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    
    # set_mode_client(custom_mode='OFFBOARD') # significance ?? --> need to send a few setpoints for setting mode to offboard mode
    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    while(not rospy.is_shutdown() and not current_state.connected):
        rospy.loginfo('Waiting for FCU connection...') # printed on terminal
        rate.sleep()

    rospy.loginfo('FCU Connected!')    

    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        # local_vel_pub.publish(vel)
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

        
        # local_vel_pub.publish(vel)

        rate.sleep()