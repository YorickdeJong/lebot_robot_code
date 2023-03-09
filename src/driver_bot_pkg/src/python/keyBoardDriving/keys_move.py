#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import motor


class Drive:
    def __init__(self):
        """Class listens to key strokes on the 'keys' topic. The velocity is set accordingly with a ramp to prevent the robot
        from slipping as a consequence of the immidiate velocity change

        Variables:
            self.key_mapping: defines directions based on key input

            self.vel_pub: publishes the velocity on the 'cmd_vel' topic

            self.vel_sub: subscribes to the key strokes, published on the 'keys' topic

            self.last_vel: velocity from the previous iteration of the while loop

            self.target_vel: target velocity based on key input 

            self.Twist_obj: object made to publish data on 'cmd_vel' topic with type Twist

            self.last_vel_send_time: last send time of the velocity

            self.vel_scales: maximum velocity 

            self.vel_ramps: acceleration towards target velocity


        """
        rospy.init_node('keys_to_twist')
        self.key_mapping = {'w': [0, 1], 'x': [0, -1],
                            'a': [-1, 0], 'd': [1, 0],
                            's': [0, 0]}

        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.vel_sub = rospy.Subscriber('keys', String, self.keys_cb)
        # sets motor throttle for the
        self.last_vel = motor.Motor()
        self.target_vel = motor.Motor()
        # publishes velocity
        self.Twist_obj = Twist()
        self.last_vel_send_time = rospy.Time.now()
        self.vel_scales = [0.8, 0.8]  # default to very slow
        self.vel_ramps = [1, 1]  # units: meters per second^2

    def ramped_vel(self, v_prev, v_target, t_prev, t_now, ramp_rate):
        """Calculates the ramp velocity  
        Args:
            v_prev: previous velocity based on self.last_vel

            v_target: target velocity based on self.target_vel

            t_prev: previous time based on self.last_vel_send_time

            t_now: current time

            ramp_rate: acceleration based on self.vel_ramps

        Returns:
            target velocity if error < step, or the previous velocity with a step * direction added

        """
        # compute maximum velocity step
        step = ramp_rate * (t_now - t_prev).to_sec()
        sign = 1.0 if (v_target > v_prev) else -1.0
        error = math.fabs(v_target - v_prev)
        print('prev vel: ' + str(v_prev))

        # we can get there within this timestep-we're done.
        if error < step:
            return v_target
        else:
            return v_prev + sign * step  # take a step toward the target

    def ramped_motor(self, prev, target, t_prev, t_now, ramps):
        """sets motor speed to calculated ramp velocity
        Args:    
            prev: previous velocity based on self.last_vel

            target: target velocity based on self.target_vel

            t_prev: previous time based on self.last_vel_send_time

            t_now: current time

            ramps: acceleration based on self.vel_ramps

        Returns:
            object tw with linear x and angular z velocity set to the ramped velocity in the robot frame

        """
        tw = motor.Motor()
        tw.set_z_angular_vel(self.ramped_vel(prev.zAngularVel, target.zAngularVel, t_prev,
                                             t_now, ramps[0]))
        tw.set_x_linear_vel(self.ramped_vel(prev.xLinearVel, target.xLinearVel, t_prev,
                                            t_now, ramps[1]))
        return tw

    def send_vel(self):
        """Publishes velocity on the 'cmd_vel' topic with message type Twister"""

        t_now = rospy.Time.now()
        self.last_vel = self.ramped_motor(self.last_vel, self.target_vel,
                                          self.last_vel_send_time, t_now, self.vel_ramps)
        self.last_vel_send_time = t_now

        # set vel from -4 to 4 as all motors work in the same direction. NB not real speed, measured in max speed
        self.Twist_obj.angular.z = self.last_vel.zAngularVel
        self.Twist_obj.linear.x = self.last_vel.xLinearVel

        self.vel_pub.publish(self.Twist_obj)

    def keys_cb(self, msg):
        """Reads keystrokes published on 'keys' topic and adjusts the target velocity accordingly
        Args:
            msg: contains information about the keystrokes

        Returns:
            target velocities in the linear x and angular z direction in the robot frame
        """

        if len(msg.data) == 0 or msg.data[0] not in self.key_mapping:
            self.target_vel.set_x_linear_vel(0)
            self.target_vel.set_z_angular_vel(0)
            return print('provide correct key')

        vels = self.key_mapping[msg.data[0]]
        self.target_vel.set_x_linear_vel(vels[1] * self.vel_scales[1])
        self.target_vel.set_z_angular_vel(vels[0] * self.vel_scales[0])


if __name__ == '__main__':
    """defines Drive() object and loops through the methods of 'drive' in order the make 
    the robot move based on key strokes published the 'keys' topic"""
    drive = Drive()
    rate = rospy.Rate(5)

    # Loop that takes in key strokes and adjusts velocity accordingly
    while not rospy.is_shutdown():
        drive.send_vel()
        rate.sleep()

    # Sets velocity to 0 upon exiting the program
    drive.last_vel.zAngularVel = 0
    drive.last_vel.xLinearVel = 0
    drive.vel_ramps = [0, 0]
    drive.send_vel()
    rate.sleep()
