

#Background

So far, the rawcrcl ROS package only frames a CRCL message (<xml> ... </xml> ). Detecting an CRCl message is not trivial as there is not an ending marker (i.e. "0" or linefeed) to detect. 

Fixme: It does not handle decipering CRCL XML . 

Of note, it is possible for mutliple clients to pump CRCL xml messages to a server. The ability to pair a client remote ip/port with the asio session is possible and tracked. However, there is reverse write mechanism. At this point, one a connection is established it assumes it is permanent and write status to it - THIS IS UNTESTED.

# Running
There is a launch file 
	roslaunch rawcrcl crclserver.launch
which allows ip and port arguments:

	roslaunch rawcrcl crclserver.launch port:=64444

This is the launch file:

	<launch>
	 <arg name="ip" default="127.0.0.1" /> 
	 <arg name="port" default="64444"  /> 
	 <node name="rawcrcl" pkg="rawcrcl" type="rawcrcl" respawn="false" output="screen" >
	   <param name="crclip" value="$(arg ip)"/>
	   <param name="crclport" value="$(arg port)"/>
	 </node>
	</launch>

Output:
Sample ROS output:
	[ INFO] [1468518803.433983207]: Crcl listen: [127.0.0.1:64444]
	Accept 127.0.0.1:47319

	[ INFO] [1468518814.538768991]: Crcl command: [<?xml version="1.0" encoding="UTF-8" standalone="yes"?><CRCLCommandInstance><CRCLCommand xsi:type="GetStatusType" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"><CommandID>1</CommandID></CRCLCommand></CRCLCommandInstance>] from 127.0.0.1:47319

You can use the javacrcl tool to quickly test connection:

After connecting with javacrcl:
michalos@rufous:rawcrcl_ws> rostopic echo /crcl_command
xml: <?xml version="1.0" encoding="UTF-8" standalone="yes"?><CRCLCommandInstance><CRCLCommand xsi:type="GetStatusType" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"><CommandID>1</CommandID></CRCLCommand></CRCLCommandInstance>
ip: 127.0.0.1
port: 47548
---

##Catkin
I suggest using catkin and not catkin_make. Info on catkin can be found at:

	http://catkin-tools.readthedocs.org/en/latest/



#Installation


# Installation of Java CRCL testing tool
## Install Java 1.08 from Oracle for Java CRCL Tool
To build java crcl tool one needs:

    JDK 1.8+ (http://www.oracle.com/technetwork/java/javase/downloads/index.html) and
    maven 3.0.5+ (https://maven.apache.org/download.cgi)

Install maven:
	$ sudo apt-get install maven


Use the command:

	mvn package

If you see this message at the beginning, bummer:

	Warning: JAVA_HOME environment variable is not set. 

You can check /usr/lib/jvm to see if a 1.8 Java Virtual Machine has been installed. If so, skip the installation step.

So you do not have Java installed. These are instructions for the less than sudo installers. Note, you need the Oracle Java JDK 1.8 version, not the 1.7 version of Ubuntu!!!

Download, unzip and copy to /usr/local/jdk1.8.0_77
or whatever is the latest 1.8 version. 

Change  you will need to change .bashrc to set the PATH to know where the jdk is installed:

	for dir in /usr/local/jdk1.8.0_77/bin /usr/lib/jvm/java-[6,7,8]-*/bin /usr/local/jdk*/bin /usr/local/jdk*/bin ; do
	  if [ -x $dir/java ] ; then
	    javadir=$dir
	  fi
	done
	if [ x"$javadir" = x ] ; then javadir=/usr/bin ; fi

	# platform-specific environment vars

	THISPLAT=`uname -s` ; export THISPLAT

	case $THISPLAT in
	    Linux)
	    PATH=$javadir:

And make sure you source the .bashrc before you attempt run.sh in java crcl. The voodoo worked for me.

THen download crcl4java: https://github.com/wshackle/crcl4java by following directions in Readme.md






