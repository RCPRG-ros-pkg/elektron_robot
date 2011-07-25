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

def cpu_monitor():
    """ 
    Function for retrieving information about current CPU usage etc.
    """
    
    diag_pub = rospy.Publisher('/diagnostics', diagnostic_msgs.msg.DiagnosticArray)
    rospy.init_node('cpu_monitor')
    
    prev_total = [0, 0, 0, 0, 0, 0, 0, 0]
    prev_idle = [0, 0, 0, 0, 0, 0, 0, 0]
    
    while not rospy.is_shutdown():
        
        #Main header
        diag = diagnostic_msgs.msg.DiagnosticArray()
        diag.header.stamp = rospy.Time.now()

        lines = open("/proc/stat", "r").readlines()
       
        p1 = subprocess.Popen(["cat", "/proc/cpuinfo"], stdout=subprocess.PIPE)
        p2 = subprocess.Popen(["grep", "MHz"], stdin=p1.stdout, stdout=subprocess.PIPE)
        
        
        p1.stdout.close()  # Allow p1 to receive a SIGPIPE if p2 exits.
        freqs = p2.communicate()[0].split("\n")


        id = 0

        for line in lines[1:]:
            jifs = line.split()
            
            if (jifs[0] == "intr"):
                break
            
            cpuname = jifs[0]
            total = 0
            idle = int(jifs[4])
            
            for jif in jifs[1:]:
                total = total + int(jif) 
            
            if (prev_total[id] < 0):
                prev_total[id] = total
                prev_idle[id] = idle
                
            diff_total = total - prev_total[id]
            diff_idle = idle - prev_idle[id]
            
            prev_total[id] = total
            prev_idle[id] = idle
                
            usage = 100.0 * (diff_total - diff_idle) / diff_total
            
            cur_freq = float(freqs[id].split(":")[1])
            max_freq = float(open("/sys/devices/system/cpu/"+cpuname+"/cpufreq/cpuinfo_max_freq", "r").readlines()[0]) / 1000
            
            # CPU info                                                                                                                              
            stat = diagnostic_msgs.msg.DiagnosticStatus()
            stat.name = cpuname
            stat.level = diagnostic_msgs.msg.DiagnosticStatus.OK
            stat.message = "OK"
        
            stat.values.append(diagnostic_msgs.msg.KeyValue("usage", str(usage)))
            stat.values.append(diagnostic_msgs.msg.KeyValue("cur_freq", str(cur_freq)))
            stat.values.append(diagnostic_msgs.msg.KeyValue("max_freq", str(max_freq)))
    
            #append
            diag.status.append(stat)
            
            # increase processor number
            id = id + 1
        
        #
        # Memory info
        #
        
        p1 = subprocess.Popen(["free","-m"], stdout=subprocess.PIPE)
        lines = p1.communicate()[0]
        
        lines = lines.split("\n")
        data = lines[1].split()
        mem_total = data[1]
        mem_used = data[2]
        mem_free = data[3]
        mem_shared = data[4]
        mem_buffers = data[5]
        mem_chached = data[6]
        # Memory info                                                                                                                              
        stat = diagnostic_msgs.msg.DiagnosticStatus()
        stat.name = "Memory"
        stat.level = diagnostic_msgs.msg.DiagnosticStatus.OK
        stat.message = "OK"
        
        stat.values.append(diagnostic_msgs.msg.KeyValue("total", mem_total))
        stat.values.append(diagnostic_msgs.msg.KeyValue("used", mem_used))
        stat.values.append(diagnostic_msgs.msg.KeyValue("free", mem_free))
        stat.values.append(diagnostic_msgs.msg.KeyValue("shared", mem_shared))
        stat.values.append(diagnostic_msgs.msg.KeyValue("buffers", mem_buffers))
        stat.values.append(diagnostic_msgs.msg.KeyValue("cached", mem_chached))
        
        
        diag.status.append(stat)
        
        #publish
        diag_pub.publish(diag)
            
            
        rospy.sleep(2.0)


if __name__ == '__main__':
    try:
        cpu_monitor()
    except rospy.ROSInterruptException: pass
    except IOError: pass
    except KeyError: pass
