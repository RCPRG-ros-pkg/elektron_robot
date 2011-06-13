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

class Gyro:

    def __init__(self):
        self.ser = serial.Serial('/dev/ttyUSB0', 38400, timeout=1)
        
        self.orientation = 0
        self.bias = 0
        
        self.frame_id = params['frame_id']
        self.prev_time = rospy.Time.now()
        
        self.pub = rospy.Publisher("/imu", Imu)

    def calibrate(self):
        pass

    def spin(self):
        while(1):
            # get line from device
            str = ser.readline()
            strs = str.split()
            pairs = []
            for s in strs:
                pairs = pairs + s.split('=')
                
            # check, if it was correct line
            if (len(pairs) != 6):
                continue
            
            sample = int(pair[5])

            # prepare Imu frame
            imu = Imu()
            imu.header.frame_id = self.frame_id
            imu.header.stamp = rospy.Time.now()
            imu.angular_velocity.x = 0.0
            imu.angular_velocity.y = 0.0
            imu.angular_velocity.z = (sample-self.bias)*math.pi/180.0
            imu.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 1]
            imu.orientation_covariance = [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.1]
            
            self.orientation += imu.angular_velocity.z * (imu.header.stamp - self.prev_time).to_sec()
            self.prev_time = imu.header.stamp
            (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w) = Rotation.RotZ(self.orientation).GetQuaternion()
            self.pub2.publish(imu)



if __name__ == '__main__':
    gyro = Gyro()
    gyro.calibrate()
    
    try:
        gyro.spin()
    except rospy.ROSInterruptException: pass
    except IOError: pass
    except KeyError: pass
