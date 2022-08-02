#!/usr/bin/env python
############################################################################
#    Copyright (C) 2009, Willow Garage, Inc.                               #
#    Copyright (C) 2013 by Ralf Kaestner                                   #
#    ralf.kaestner@gmail.com                                               #
#    Copyright (C) 2013 by Jerome Maye                                     #
#    jerome.maye@mavt.ethz.ch                                              #
#                                                                          #
#    All rights reserved.                                                  #
#                                                                          #
#    Redistribution and use in source and binary forms, with or without    #
#    modification, are permitted provided that the following conditions    #
#    are met:                                                              #
#                                                                          #
#    1. Redistributions of source code must retain the above copyright     #
#       notice, this list of conditions and the following disclaimer.      #
#                                                                          #
#    2. Redistributions in binary form must reproduce the above copyright  #
#       notice, this list of conditions and the following disclaimer in    #
#       the documentation and/or other materials provided with the         #
#       distribution.                                                      #
#                                                                          #
#    3. The name of the copyright holders may be used to endorse or        #
#       promote products derived from this software without specific       #
#       prior written permission.                                          #
#                                                                          #
#    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   #
#    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     #
#    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     #
#    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        #
#    COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  #
#    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  #
#    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      #
#    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      #
#    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    #
#    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN     #
#    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       #
#    POSSIBILITY OF SUCH DAMAGE.                                           #
############################################################################

from __future__ import with_statement

import rospy

import traceback
import threading
import sys
from time import sleep
import subprocess
import re

import socket

from diagnostic_msgs.msg import KeyValue
from er_robot_status_msgs.msg import NetworkUsageInfo, NetworkInterfaceInfo

net_level_warn = 0.95
net_capacity = 128

stat_dict = {0: 'OK', 1: 'Warning', 2: 'Error'}


def get_sys_net_stat(iface, sys):
    cmd = 'cat /sys/class/net/%s/statistics/%s' % (iface, sys)
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
    stdout, stderr = p.communicate()
    return (p.returncode, stdout.strip())


def get_sys_net(iface, sys):
    cmd = 'cat /sys/class/net/%s/%s' % (iface, sys)
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
    stdout, stderr = p.communicate()
    return (p.returncode, stdout.strip())


class NetMonitor():
    def __init__(self, hostname, diag_hostname):
        self.network_usage_pub = rospy.Publisher('/network_usage', NetworkUsageInfo, queue_size=100)

        self._mutex = threading.Lock()

        self._net_level_warn = rospy.get_param('~net_level_warn', net_level_warn)
        self._net_capacity = rospy.get_param('~net_capacity', net_capacity)

        self.network_usage_stats = NetworkUsageInfo()
        self.network_usage_stats.hostname = hostname
        self.network_usage_stats.level = NetworkUsageInfo.UNKNOWN
        self.network_usage_stats.diagnostics.append(KeyValue(key='Level Mapping', value=NetworkUsageInfo.LEVELSTR))
        self.network_usage_stats.interfaces = []
        self.interfaces = {}

        self._last_usage_time = 0
        self._last_publish_time = 0

    def check_network(self):
        self.network_usage_stats = NetworkUsageInfo()
        self.network_usage_stats.stamp = rospy.get_rostime()
        self.network_usage_stats.hostname = hostname
        self.network_usage_stats.level = NetworkUsageInfo.UNKNOWN
        self.network_usage_stats.diagnostics.append(KeyValue(key='Level Mapping', value=NetworkUsageInfo.LEVELSTR))
        self.network_usage_stats.interfaces = []
        try:
            p = subprocess.Popen('ifstat -q -S 1 1', stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
            stdout, stderr = p.communicate()

            retcode = p.returncode
            if retcode != 0:
                self.network_usage_stats.level = NetworkUsageInfo.ERROR
                self.network_usage_stats.interfaces = []
                self.network_usage_stats.diagnostics.append(
                    KeyValue(key="\"ifstat -q -S 1 1\" Call Error", value=str(retcode)))
                return

            rows = stdout.split('\n')
            data = rows[0].split()
            ifaces = []
            for i in range(0, len(data)):
                ifaces.append(data[i])

            data = rows[2].split()
            kb_in = []
            kb_out = []
            for i in range(0, len(data), 2):
                kb_in.append(data[i])
                kb_out.append(data[i + 1])

            self.network_usage_stats.level = NetworkUsageInfo.OK
            for i in range(0, len(ifaces)):
                self.interfaces[ifaces[i]] = NetworkInterfaceInfo()
                self.interfaces[ifaces[i]].stamp = rospy.get_rostime()
                self.interfaces[ifaces[i]].name = ifaces[i]

                (retcode, cmd_out) = get_sys_net(ifaces[i], 'operstate')
                if retcode == 0:
                    self.interfaces[ifaces[i]].state = cmd_out
                    ifacematch = re.match('eth[0-9]+', ifaces[i])
                    if ifacematch and (cmd_out == 'down' or cmd_out == 'dormant'):
                        self.network_usage_stats.level = NetworkUsageInfo.ERROR

                self.interfaces[ifaces[i]].input_traffic_MBps = float(kb_in[i]) / 1024
                self.interfaces[ifaces[i]].output_traffic_MBps = float(kb_out[i]) / 1024

                net_usage_in = float(kb_in[i]) / 1024 / self._net_capacity
                net_usage_out = float(kb_out[i]) / 1024 / self._net_capacity
                if net_usage_in > self._net_level_warn or net_usage_out > self._net_level_warn:
                    self.network_usage_stats.level = NetworkUsageInfo.WARN
                    self.interfaces[ifaces[i]].level = NetworkInterfaceInfo.HIGH_NETWORK_USAGE

                (retcode, cmd_out) = get_sys_net(ifaces[i], 'mtu')
                if retcode == 0:
                    self.interfaces[ifaces[i]].mtu = cmd_out

                (retcode, cmd_out) = get_sys_net_stat(ifaces[i], 'rx_bytes')
                if retcode == 0:
                    self.interfaces[ifaces[i]].total_received_MB = float(
                        cmd_out) / 1024 / 1024

                (retcode, cmd_out) = get_sys_net_stat(ifaces[i], 'tx_bytes')
                if retcode == 0:
                    self.interfaces[ifaces[i]].total_transmitted_MB = float(
                        cmd_out) / 1024 / 1024

                (retcode, cmd_out) = get_sys_net_stat(ifaces[i], 'collisions')
                if retcode == 0:
                    self.interfaces[ifaces[i]].collisions = int(cmd_out)

                (retcode, cmd_out) = get_sys_net_stat(ifaces[i], 'rx_errors')
                if retcode == 0:
                    self.interfaces[ifaces[i]].rx_errors = int(cmd_out)

                (retcode, cmd_out) = get_sys_net_stat(ifaces[i], 'tx_errors')
                if retcode == 0:
                    self.interfaces[ifaces[i]].tx_errors = int(cmd_out)

                self.network_usage_stats.interfaces.append(self.interfaces[ifaces[i]])

            # self.network_usage_stats.interfaces = [interface for _, interface in self.interfaces]

        except Exception as e:
            rospy.logerr(traceback.format_exc())
            msg = 'Network Usage Check Error'
            self.network_usage_stats.level = NetworkUsageInfo.ERROR
            self.network_usage_stats.diagnostics.append(KeyValue(key=msg, value=str(e)))

    def publish_stats(self):
        self.check_network()
        self.network_usage_pub.publish(self.network_usage_stats)
        self._last_publish_time = rospy.get_time()


if __name__ == '__main__':
    hostname = socket.gethostname()
    hostname = hostname.replace('-', '_')

    import optparse
    parser = optparse.OptionParser(usage="usage: net_monitor.py [--diag-hostname=cX]")
    parser.add_option("--diag-hostname", dest="diag_hostname",
                      help="Computer name in diagnostics output (ex: 'c1')",
                      metavar="DIAG_HOSTNAME",
                      action="store", default=hostname)
    options, args = parser.parse_args(rospy.myargv())
    try:
        rospy.init_node('net_monitor_%s' % hostname)
    except rospy.exceptions.ROSInitException:
        print >> sys.stderr, 'Network monitor is unable to initialize node. Master may not be running.'
        sys.exit(0)
    net_node = NetMonitor(hostname, options.diag_hostname)
    rate = rospy.Rate(1.0)
    try:
        while not rospy.is_shutdown():
            rate.sleep()
            net_node.publish_stats()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        traceback.print_exc()
        rospy.logerr(traceback.format_exc())

    sys.exit(0)
