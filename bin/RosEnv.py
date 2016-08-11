import os
import subprocess

def PipedCmd(cmd):
        task = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE )
        data = task.stdout.read()
#        data1 = task.stderr.read()
        return data

def RosEnvironment()
	dict = {}
	cmd = '/bin/bash -i /usr/local/michalos/nistcrcl_ws/src/nistcrcl/scripts/setup.bash'
	envs = ExecuteShellCommand(cmd);
	lines = envs. splitlines ()
	for line in lines:
		key, val = line.strip().split('=')
		dict[key.strip()] = val.strip()
	for key in dict:	
		os.environ[key]=dict[key]
