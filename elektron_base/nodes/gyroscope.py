#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import roslib
roslib.load_manifest('elektron_base')
import rospy
import math
from sensor_msgs.msg import Imu
import serial
from PyKDL import Rotation

class Gyro:

    def __init__(self):
        self.device = rospy.get_param('~device', '/dev/ttyUSB0')
        self.baud = rospy.get_param('~baud', 38400)
        self.ser = serial.Serial(self.device, self.baud, timeout=1)

        self.orientation = 0
        self.bias = 0

        self.frame_id = 'base_footprint'
        self.prev_time = rospy.Time.now()

        self.pub = rospy.Publisher("/imu_data", Imu)
        
        self.scale = 360.0 / 383.2

    def calibrate(self):
        rospy.loginfo("Calibrating Gyro. Don't move the robot now")
        start_time = rospy.Time.now()
        cal_duration = rospy.Duration(4.0)
        offset = 0
        cnt = 0
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
        pass

    def spin(self):
        self.prev_time = rospy.Time.now()

        while(1):
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
