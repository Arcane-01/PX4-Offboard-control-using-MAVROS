#!/usr/bin/python3

# Update: Added a publisher to publish desired state for the quadrotor in the main loop
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

# list for plotting the roll error
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


class QuadrotorController: # class for Quadrotor Control

    def __init__(self):
        # rospy.init_node('quadrotor_controller')
        self.attitude_command_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        self.position_command_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10) # currently unused 
        # self.imu_sub = rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback) # subscriber to get data from IMU

        # the sample time
        # self.dt = 0.1

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
    def get_roll(self, current_pose):
        # Get current roll angle from the orientation quaternion
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
        global roll_error_arr

        # Get current roll angle from the orientation quaternion
        roll = self.get_roll(current_pose)

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

    def publish_roll_command(self, roll_command):
        attitude_target = AttitudeTarget()
        attitude_target.header.stamp = rospy.Time.now()
        attitude_target.type_mask = (
            AttitudeTarget.IGNORE_PITCH_RATE
            | AttitudeTarget.IGNORE_YAW_RATE
            | AttitudeTarget.IGNORE_THRUST
        )
        attitude_target.body_rate.x = roll_command
        attitude_target.body_rate.y = 0.0
        attitude_target.body_rate.z = 0.0

        self.attitude_command_pub.publish(attitude_target)

    def publish_position_command(self, position_command): # currently unused 

        position_target = PositionTarget()
        position_target.header.stamp = rospy.Time.now()
        position_target.coordinate_frame = PositionTarget.FRAME_BODY_NED
        position_target.type_mask = ( PositionTarget.IGNORE_VX | 
                                      PositionTarget.IGNORE_VY | 
                                      PositionTarget.IGNORE_VZ | 
                                      PositionTarget.IGNORE_AFX | 
                                      PositionTarget.IGNORE_AFY | 
                                      PositionTarget.IGNORE_AFZ | 
                                      PositionTarget.IGNORE_YAW | 
                                      PositionTarget.IGNORE_YAW_RATE )
        #  For position x, y, z
        position_target.position.x = 0.0
        position_target.position.y = 0.0
        position_target.position.z = 0.0

        # For velocities vx, vy, vz
        position_target.velocity.x = 0.0
        position_target.velocity.y = 0.0
        position_target.velocity.z = 0.0
        

        position_target.acceleration_or_force.x = 0.0
        position_target.acceleration_or_force.y = 0.0
        position_target.acceleration_or_force.z = 0.0
        
        position_target.yaw = 0.0
        position_target.yaw_rate = 0.0

        self.position_command_pub.publish(position_target)

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.compute_roll_control()
            # self.publish_position_command()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("my_controller_py")

    # Subscribers for state, position and velocity
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=pose_cb) # used for finding the current roll of the quadrotor 
    velocity_sub = rospy.Subscriber("mavros/local_position/velocity", TwistStamped, callback=velocity_cb) # can be used when controller is applied for velocities | currently unused

    # Get position and orientation of the drone | currently unused
    pos = current_pose.pose.position
    ori = current_pose.pose.orientation
    
    # Publishers for position and velocity
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    # local_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)

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

    desired_pose = PoseStamped()

    desired_pose.pose.position.x = 0.0
    desired_pose.pose.position.y = 0.0
    desired_pose.pose.position.z = 1.0

    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(desired_pose)
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

        controller = QuadrotorController()
        controller.run()

        # local_pos_pub.publish(desired_pose)

        rate.sleep()