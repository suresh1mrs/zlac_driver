import logging
from math import *
import numpy as np
import rospy
from dataclasses import dataclass

@dataclass(frozen=False)
class ODOM:
    """
    Dataclass to represent Odometry in python.
    """
    x = 0.0
    y = 0.0
    yaw = 0.0
    v = 0.0
    w = 0.0

    def __str__(self):
        return f"x:{self.x}, y:{self.y}, yaw:{self.yaw}, v:{self.v}, w:{self.w}"

class DiffDrive:
    """Differential drive kinematics for 2-wheel robot.
    """
    def __init__(self, wheel_radius, track_width):
        self._wheel_radius = wheel_radius
        self._track_width = track_width 
        self._odom = ODOM()

        # self.path = "None"

        # self._l_pos = 0 # Left encoder position
        # self._r_pos = 0 # Right right
        # self._l_vel = 0 # left wheel linear velocity in m/s
        # self._r_vel = 0 # right wheel linear velocity in m/s

        self.startup_flag = True

        self.dist_l = 0
        self.dist_r = 0

        self.prev_dist_l = 0
        self.prev_dist_r = 0

    def calcWheelVel(self,v,w):
        """Calculates the left and right wheel speeds in rad/s from vx and w

        Parameters
        --
        @param v linear/forward velocity (v_x) in robot frame, m/s
        @param w Robot's angular velocity in rad/s

        Returns
        --
        @return wl Left wheel velocity in rad/s
        @return wr Right wheel velocity in rad/s
        """
        wr = 1/self._wheel_radius *(v + w * self._track_width/2)
        wl = 1/self._wheel_radius *(v - w * self._track_width/2)
        return (wl,wr)
    
    def calcRobotOdom(self, dt):
        """calculates linear and angular states from the left and right wheel speeds

        Parameters
        --
        @param dt time stamp in seconds 
        """

        if self.startup_flag:
            self.prev_dist_l = self.dist_l
            self.prev_dist_r = self.dist_r

            self.startup_flag = False

        # vl = self._l_vel
        # vr = self._r_vel

        dist_l = self.dist_l
        dist_r = self.dist_r

        # Left wheel returns increasing value 
        delta_l = dist_l - self.prev_dist_l
        # Right wheels returns decreasing value
        delta_r = self.prev_dist_r - dist_r

        self.prev_dist_l = self.dist_l
        self.prev_dist_r = self.dist_r

        th = (delta_r - delta_l)/ self._track_width

        mean_dist = (delta_r + delta_l)/2
        self._odom.v = mean_dist/dt
        self._odom.w = th/dt

        self._odom.yaw += th

        self._odom.x += mean_dist * np.cos(self._odom.yaw)
        self._odom.y += mean_dist * np.sin(self._odom.yaw)


        # rospy.loginfo(f"Velocity from driver, vl:{vl}, vr:{vr}")
        # rospy.loginfo(f"Distance from driver, dist_l:{self.dist_l}, dist_r:{self.dist_r}")
        # rospy.loginfo(f"prev dist, prev_dist_l:{self.prev_dist_l}, prev_dist_r:{self.prev_dist_r}")

        # try:
        #     if (vl > 0.0) and (vr < 0.0) and (abs(vl) == abs(vr)):
        #         ## rotatiing CW
        #         linear_vel = 0
        #         angular_vel = (vl-vr)/self._track_width
        #         self._odom['yaw'] = self._odom['yaw'] - angular_vel*dt

        #         self.path = "skid_right"

        #     elif (vr > 0.0) and (vl < 0.0) and (abs(vl) == abs(vr)):
        #         ## rotatiing CCW
        #         linear_vel = 0
        #         angular_vel = (vr-vl)/self._track_width
        #         self._odom['yaw'] = self._odom['yaw'] + angular_vel*dt

        #         self.path = "skid_left"

        #     elif abs(vl) > abs(vr):
        #         ## curving CW
        #         linear_vel = (vl + vr)/2.0
        #         angular_vel = (vl-vr)/self._track_width
        #         R_ICC = (self._track_width/2.0)*((vl+vr)/(vl-vr))

        #         self._odom['x'] = self._odom['x'] - R_ICC*np.sin(self._odom['yaw']) + R_ICC*np.sin(self._odom['yaw'] + angular_vel*dt)
        #         self._odom['y'] = self._odom['y'] + R_ICC*np.cos(self._odom['yaw']) - R_ICC*np.cos(self._odom['yaw'] + angular_vel*dt)
        #         self._odom['yaw'] = self._odom['yaw'] - angular_vel*dt

        #         self.path = "curve_right"

        #     elif abs(vl) < abs(vr):
        #         ## curving CCW
        #         linear_vel = (vl + vr)/2.0
        #         angular_vel = (vr-vl)/self._track_width
        #         R_ICC = (self._track_width/2.0)*((vr+vl)/(vr-vl))

        #         self._odom['x'] = self._odom['x'] - R_ICC*np.sin(self._odom['yaw']) + R_ICC*np.sin(self._odom['yaw'] + angular_vel*dt)
        #         self._odom['y'] = self._odom['y'] + R_ICC*np.cos(self._odom['yaw']) - R_ICC*np.cos(self._odom['yaw'] + angular_vel*dt)
        #         self._odom['yaw'] = self._odom['yaw'] + angular_vel*dt

        #         self.path = "curve_left"

        #     elif vl == vr:
        #         linear_vel = (vl + vr)/2.0
        #         angular_vel = 0.0
        #         self._odom['x'] = self._odom['x'] + linear_vel*np.cos(self._odom['yaw'])*dt
        #         self._odom['y'] = self._odom['y'] + linear_vel*np.sin(self._odom['yaw'])*dt
        #         self._odom['yaw'] = self._odom['yaw']
        #         self.path = "straight"

        #     else:
        #         linear_vel = 0.0
        #         angular_vel = 0.0
        #         R_ICC = 0.0
        # except Exception as e:
        #         rospy.logerr_throttle(1, "[Odom Calculation] Error in Calculation: %s", e)

        # # Update odometry
        # self._odom['v'] = linear_vel
        # self._odom['w'] = angular_vel

        # rospy.loginfo(f"Odometry Computed {self._odom}")
        # rospy.loginfo(f"Guddu odom, x:{self.x}, y:{self.y}, yaw:{self.robot_angle}, v:{vx} w:{vth}")


        return self._odom

    def resetOdom(self):
        """Reset Odom to origin
        """
        self._odom = ODOM()