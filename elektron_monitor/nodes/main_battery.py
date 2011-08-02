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
roslib.load_manifest('elektron_monitor')

import rospy

import diagnostic_msgs.msg
import serial
import math


from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class Battery:

    def __init__(self):
        # open serial port
        self.device = rospy.get_param('~device', '/dev/ttyUSB3')
        self.baud = rospy.get_param('~baud', 9600)
        self.ser = serial.Serial(self.device, self.baud, timeout=0.3)
        
        self.diag_pub = rospy.Publisher('/diagnostics', diagnostic_msgs.msg.DiagnosticArray)
        
        self.adc_factor = 9.34 / 307
        self.last_voltage = 0.0
        
        self.last_beep = rospy.Time.now()
        self.lvl_low = 21
        self.lvl_crit = 19.5
        
        self.play_music = rospy.get_param('~play_music', False)
        
        if (self.play_music):
            self.soundhandle = SoundClient()
            self.snd_low = rospy.get_param('~snd_low', '')
            self.snd_crit = rospy.get_param('~snd_critical', '')
            self.snd_plugin = rospy.get_param('~snd_plugin', '')
            self.snd_plugout = rospy.get_param('~snd_plugout', '')

    def spin(self):

        self.ser.flushInput()

        while not rospy.is_shutdown():

            # get data
            s = self.ser.read(2)
            
            #Main header
            diag = diagnostic_msgs.msg.DiagnosticArray()
            diag.header.stamp = rospy.Time.now()
    
            
            #battery info                                                                                                                              
            stat = diagnostic_msgs.msg.DiagnosticStatus()
            stat.name = "Main Battery"
            stat.level = diagnostic_msgs.msg.DiagnosticStatus.OK
            stat.message = "OK"

            # check, if it was correct line
            if (len(s) < 2):
                continue
            
            hi = ord(s[0])
            lo = ord(s[1])
            voltage = (hi*256 + lo) * self.adc_factor
            
            if (voltage < self.lvl_crit) and (rospy.Time.now() - self.last_beep > rospy.Duration(30)):
                rospy.logwarn("Critical power level.")
                self.last_beep = rospy.Time.now()
                if (self.play_music):
                    self.soundhandle.playWave(self.snd_crit)
            else:
                if (voltage < self.lvl_low) and (rospy.Time.now() - self.last_beep > rospy.Duration(120)):
                    rospy.logwarn("Low power level.")
                    self.last_beep = rospy.Time.now()
                    if (self.play_music):
                        self.soundhandle.playWave(self.snd_low)
                        
            # Just plugged in
            if (self.last_voltage < 26) and (voltage > 28):
                rospy.loginfo("Power charger plugged in.")
                if (self.play_music):
                    self.soundhandle.playWave(self.snd_plugin)
            
            # Just plugged out
            if (self.last_voltage > 28) and (voltage < 26):
                rospy.loginfo("Power charger plugged out.")
                if (self.play_music):
                    self.soundhandle.playWave(self.snd_plugout)
            
            self.last_voltage = voltage
            stat.values.append(diagnostic_msgs.msg.KeyValue("Voltage", str(voltage)))

            #append
            diag.status.append(stat)
            
            #publish
            self.diag_pub.publish(diag)



if __name__ == '__main__':
    rospy.init_node('main_battery_monitor')      
    bat = Battery()

    try:
        bat.spin()
    except rospy.ROSInterruptException: pass
    except IOError: pass
    except KeyError: pass
