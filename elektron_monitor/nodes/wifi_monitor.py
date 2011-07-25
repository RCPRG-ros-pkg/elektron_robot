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

import subprocess
import re

def wifi_monitor():
    """ 
    Function for retrieving information about wireless network interfaces,
    publishing signal power and noise ratios, IP address etc. 
    """
    
    diag_pub = rospy.Publisher('/diagnostics', diagnostic_msgs.msg.DiagnosticArray)
    rospy.init_node('wifi_monitor')
    while not rospy.is_shutdown():
        
        #Main header
        diag = diagnostic_msgs.msg.DiagnosticArray()
        diag.header.stamp = rospy.Time.now()

        lines = open("/proc/net/wireless", "r").readlines()

        faces = {}
        for line in lines[2:]:
            if line.find(":") < 0: continue
            face, data = line.split(":")
            strs = data.split()
            link = float(strs[1])
            level = float(strs[2])
            noise = float(strs[3])
            face = face.strip()
        
            p1 = subprocess.Popen(["ifconfig", face], stdout=subprocess.PIPE)
            p2 = subprocess.Popen(["grep", "inet addr"], stdin=p1.stdout, stdout=subprocess.PIPE)
            p3 = subprocess.Popen(["awk", "-F:", "{print $2}"], stdin=p2.stdout, stdout=subprocess.PIPE)
            p4 = subprocess.Popen(["awk",  "{print $1}"], stdin=p3.stdout, stdout=subprocess.PIPE)
            
            p1.stdout.close()  # Allow p1 to receive a SIGPIPE if p2 exits.
            p2.stdout.close()  # Allow p1 to receive a SIGPIPE if p3 exits.
            p3.stdout.close()  # Allow p1 to receive a SIGPIPE if p4 exits.
            output = p4.communicate()[0]
        
            #rospy.logwarn("Output: %s"%output)   
            
            output = subprocess.Popen(['ifconfig', face], stdout=subprocess.PIPE).communicate()[0]
            ret = re.findall('inet addr:([^ ]*) ', output)
            if (len(ret) > 0):
                ip = ret[0]
            else:
                ip = "not connected"  
        
            #rospy.logwarn("Output: %s"%ip)
        
            #interface info                                                                                                                              
            stat = diagnostic_msgs.msg.DiagnosticStatus()
            stat.name = face
            stat.level = diagnostic_msgs.msg.DiagnosticStatus.OK
            stat.message = "OK"
        

            stat.values.append(diagnostic_msgs.msg.KeyValue("Quality", str(link)))
            stat.values.append(diagnostic_msgs.msg.KeyValue("Level", str(level)))
            stat.values.append(diagnostic_msgs.msg.KeyValue("Noise", str(noise)))
            stat.values.append(diagnostic_msgs.msg.KeyValue("IP", ip))

            #append
            diag.status.append(stat)
        
        #publish
        diag_pub.publish(diag)
        rospy.sleep(2.0)


if __name__ == '__main__':
    try:
        wifi_monitor()
    except rospy.ROSInterruptException: pass
    except IOError: pass
    except KeyError: pass
