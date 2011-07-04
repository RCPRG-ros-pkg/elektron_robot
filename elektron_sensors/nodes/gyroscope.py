#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Robot Control and Pattern Recognition Group, Warsaw University of Technology
#
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the <organization> nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import roslib
roslib.load_manifest('elektron_sensors')

import rospy

from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyResponse

from PyKDL import Rotation

import serial
import math


class Gyro:
    """ 
    Class for interfacing with gyroscope on Elektron mobile robot.
    Responsible for retrieving data from device, calibration and
    calculating current orientation. 
    """

    def __init__(self):
        # open serial port
        self.device = rospy.get_param('~device', '/dev/ttyUSB0')
        self.baud = rospy.get_param('~baud', 38400)
        self.ser = serial.Serial(self.device, self.baud, timeout=1)

        # reset variables
        self.orientation = 0
        self.bias = 0

        self.calibrating = False

        self.frame_id = 'base_footprint'
        self.prev_time = rospy.Time.now()

        # publisher with imu data
        self.pub = rospy.Publisher("/imu/data", Imu)
        
        # rotation scale
        self.scale = rospy.get_param('~rot_scale', 1.0)
        
        # service for calibrating gyro bias
        rospy.Service("/imu/calibrate", Empty, self.calibrateCallback)
        
        # publisher with calibration state
        self.is_calibratedPublisher = rospy.Publisher('/imu/is_calibrated', Bool, latch=True)
        
        # We'll always just reuse this msg object:        
        self.is_CalibratedResponseMsg = Bool();

        # Initialize the latched is_calibrated state. 
        # At the beginning calibration is assumed to be done

        self.is_CalibratedResponseMsg.data = True;
        self.is_calibratedPublisher.publish(self.is_CalibratedResponseMsg)

    def calibrate(self):
        # calibration routine
        rospy.loginfo("Calibrating Gyro. Don't move the robot now")
        start_time = rospy.Time.now()
        cal_duration = rospy.Duration(5.0)
        offset = 0
        cnt = 0
        # flush input buffer to calibrate on newest data  
        self.ser.flushInput()
        while rospy.Time.now() < start_time + cal_duration:

            # get line from device
            str = self.ser.readline()
            nums = str.split()

            # check, if it was correct line
            if (len(nums) != 5):
                continue

            cnt += 1
            gyro = int(nums[2])
            ref = int(nums[3])
            temp = int(nums[4])

            val = ref-gyro;

            offset += val;

        self.bias = 1.0 * offset / cnt
        rospy.loginfo("Gyro calibrated with offset %f"%self.bias)
        
        # Update the latched is_calibrated state:
        self.is_CalibratedResponseMsg.data = True
        self.is_calibratedPublisher.publish(self.is_CalibratedResponseMsg)
        
        return True



    def calibrateCallback(self, req):
        """The imu/calibrate service handler."""
          
        rospy.loginfo("Calibration request")
                
        self.calibrating = True
        
        return EmptyResponse()
      

    def spin(self):
        self.prev_time = rospy.Time.now()

        while not rospy.is_shutdown():
            if self.calibrating:
                self.calibrate()
                self.calibrating = False
                self.prev_time = rospy.Time.now()
            
            # prepare Imu frame
            imu = Imu()
            imu.header.frame_id = self.frame_id

            # get line from device
            str = self.ser.readline()

            # timestamp
            imu.header.stamp = rospy.Time.now()

            nums = str.split()

            # check, if it was correct line
            if (len(nums) != 5):
                continue

            gyro = int(nums[2])
            ref = int(nums[3])
            temp = int(nums[4])

            val = (ref-gyro - self.bias) * 1000 / 3 / 1024 * self.scale

            imu.angular_velocity.x = 0
            imu.angular_velocity.y = 0
            imu.angular_velocity.z = val * math.pi / 180
            imu.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 1]
            imu.orientation_covariance = [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.1]

            self.orientation += imu.angular_velocity.z * (imu.header.stamp - self.prev_time).to_sec()
            self.prev_time = imu.header.stamp
            (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w) = Rotation.RotZ(self.orientation).GetQuaternion()
            self.pub.publish(imu)



if __name__ == '__main__':
    rospy.init_node('gyroscope')
    gyro = Gyro()
    gyro.calibrate()

    try:
        gyro.spin()
    except rospy.ROSInterruptException: pass
    except IOError: pass
    except KeyError: pass
