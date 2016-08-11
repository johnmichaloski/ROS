#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
DOESN"T WORK :()
Directions:
1. Set up environment bash file and set to cmd
   #!/bin/bash
   env 
2. Pass this python file to the folder you are working in to RosEnvironment() after devel/setup.bash
   e.g., '/bin/bash -i /usr/local/michalos/nistcrcl_ws/src/nistcrcl/scripts/setup.bash'
3. import RosEnv
4. After main, you need to setup ROS environment or 
if __name__ == '__main__':
    RosEnv.RosEnvironment(bash cmd string)
'''
import os
import subprocess


def ExecuteShellCommand(cmd):
        task = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE )
        data = task.stdout.read()
#        data1 = task.stderr.read()
        return data
def RosEnvironment(cmd):
    dict = {}
    envs = ExecuteShellCommand(cmd);
    print envs
    lines = envs. splitlines ()
    for line in lines:
        print "line=",line
        key, val = line.strip().split('=',1)
        dict[key.strip()] = val.strip()
    for key in dict:	
        os.environ[key]=dict[key]
