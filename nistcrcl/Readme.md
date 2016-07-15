
#Readme for Conversion of CRCL XML into ROS Message 
----

Michaloski, John L.
5/30/2016 2:00:00 PM
NistCrclReadme.docx

This document presents.
#Background
First, the nistcrcl ROS package frames a CRCL message (). Detecting an CRCl message is not trivial as there is not an ending marker (i.e. "0" or linefeed) to detect. If a CRCl XML message is detected, it is placed onto a synchronized queue. Another thread will look for CRCL messages on this queue. If it finds a CRCL message, it interprets this message by translating CRCL robot data structures (e.g., joint and pose) into ROS representation. Finally, after the message has been decoded into canonical ROS command, it is published onto a ROS message. .
#Running
There is a launch file roslaunch nistcrcl crclserver.launch which allows ip and port arguments:
	roslaunch nistcrcl crclserver.launch port:=64444


##How does the Crcl2Ros class work?
	in the main cpp program we declare the Crcl2Ros class  (with ROS node handle reference and cycle time parameter of10 milliseconds).
	// This thread handles new XML messages received from  crcl asio socket.
	CCrcl2Ros crcl2ros(nh,10);
	session.Start(); // start the thread
It uses the Thread template to run cyclically. First it calls Init() and then cyclically it call Action() both virtual overloaded methods.
	 class Thread
	       void Cycle ( )
	        {
	            Init( ); <--- calls ControllerSession::Init() 
	            _timer.sync( );
	
	            while ( _bThread )
	            {
	                Action( );
	                _timer.wait( );
	            }
	
	            Cleanup( );
	        }
It uses the Thread template to run cyclically.
	void CCrcl2RosMsg::Init() {
	
	    Name() = "CCrcl2RosMsg";
	    crclinterface = boost::shared_ptr<Crcl::CrclDelegateInterface>(
	            new Crcl::CrclDelegateInterface());
	    crclinterface->SetAngleUnits("DEGREE");
	    crcl_pub = _nh.advertise<nistcrcl::CrclCommandMsg>("crcl_command", 10);
	    // Reading status not done yet
	//    crcl_sub = _nh.subscribe("crcl_status", 1000, crclstatusCallback);
	}

In the Action() method determines if new CRCL command has been placed on the message queue. 
	        if (CAsioCrclSession::InMessages().SizeMsgQueue() > 0) {
	            CrclMessage msg = CAsioCrclSession::InMessages().PopFrontMsgQueue();
	            std::string crclcmd = boost::get<0>(msg);
	            _pSession = boost::get<1>(msg);
If so, the CRCL command (and CRCL data structures) is interpreted and if necessary translated into a ROS command (with ROS data structures). 
	          Crcl::CrclReturn ret = crclinterface->DelegateCRCLCmd(crclcmd);

If the return code indicates that the CRCL command is requesting a status message, the latest local CRCL world model is wrapped into a CRCL status reply:
	  if (ret == Crcl::CANON_STATUSREPLY) {
	  // Generate status reply message
	  std::string sStatus = Crcl::CrclClientCmdInterface().GetStatusReply(&crclinterface->crclwm);
	  _pSession->SyncWrite(sStatus);
	  }
	 }

Next these canonical "ROS" commands are translated into a ROS crcl message.  ROS requires standard units (meter, radians) while CRCL allows mm, angles, etc. So the commands have to be translated to be interpreted. First, the cmds queue is checked for new commands.        
	        if (RCS::cmds.SizeMsgQueue() > 0) {
	            RCS::CanonCmd cc = RCS::cmds.PopFrontMsgQueue();
If one is found, then a new ROS message is created 
	  nistcrcl::CrclCommandMsg rosmsg;
And the contents of this union of all CRCL commands (excluding tolerance, velocity, acceleration, force parameters which is omitted at this time.)
	   if (cc.cmd == RCS::CANON_MOVE_JOINT)
	   {
	    rosmsg.crclcommand=actuatejoints;
	     rosmsg.joints=cc.joints;
	   }
	   else if (cc.cmd == RCS::CANON_MOVE_TO)
	   {
	    rosmsg.crclcommand=moveto;
	    rosmsg.finalpose.position.x=cc.pose.getOrigin().x();
	    rosmsg.finalpose.position.y=cc.pose.getOrigin().y();
	    rosmsg.finalpose.position.z=cc.pose.getOrigin().z();
	    rosmsg.finalpose.orientation.x=cc.pose.getRotation().x();
	    rosmsg.finalpose.orientation.y=cc.pose.getRotation().y();
	    rosmsg.finalpose.orientation.z=cc.pose.getRotation().z() ;
	    rosmsg.finalpose.orientation.w=cc.pose.getRotation().w();
	   }
	. . .
Again, numerous ROS representations of pose position and orientation exist, To use existing ROS message structure the geometry pose (with point and quaternion sub definition) was used. Readers are referred to https://github.com/ros/common_msgs to view a list of common ROS messages.
Finally, if a ROS message was generated it is published on the topic (nistcrcl/ crcl_command):
	   // publish ros message if found corresponding crcl command
	   if(rosmsg.crclcommand!=noop)
	   {
	    ROS_INFO("ROS command: [%s] ", rosmsg.crclcommand);
	    crcl_pub.publish(rosmsg);
	   }


#Canonical Robot Control Language (CRCL) Background 
CRCL is part of the robot research at NIST for Kit building. Kit building or "kitting" is a process in which individually separate but related items are grouped, packaged, and supplied together as one unit (kit). To pick and place parts and components during kitting, the kitting workcell relies on a sensor system to retrieve the six-degree of freedom (6DOF) pose estimation of each of these objects. While the use of a sensor system allows objects' poses to be obtained, it also helps detecting failures during the execution of a kitting plan when some of these objects are missing or are not at the expected locations. 
Ontologies are a formal way to describe taxonomies and classification networks, essentially defining the structure of knowledge for various domains: the nouns representing classes of objects and the verbs representing relations between the objects [5].  The kitting ontology has been fully defined in OWL. However, the ontology was also fully defined in the XML schema language.  Tools to transform XML Schema file in Ontology definition files were developed [6] [7].
This kitting ontology models commands expressed in the canonical robot command language (CRCL). CRCL is a messaging language for controlling a robot. CRCL commands are executed by a low-level device robot controller. The usual source of CRCL commands is a plan/program execution system. CRCL is intended for use with devices typically described as industrial robots and for other automated positioning devices such as automated guided vehicles (AGVs). An AGV with a robotic arm attached may be regarded as a single robot responding to a single stream of CRCL commands or as two robots responding to two separate streams of CRCL commands. Although CRCL is not a programming language, the commands are in the context of a session consisting of getting ready for activity, performing activities, and becoming quiescent. CRCL commands may be collected in files for testing purposes, but executing such files (by giving the commands in the order they occur in the file) is not be the normal operating mode of a robot. Because robots operate in uncertain and changing environment, the reliance on sensors to adjust for such disturbances makes canned scripts ineffective under real conditions.
MTConnect is primarily a status reporting mechanism, so that the bulk of the work is fetching and reporting the status of CRCL robots. CRCL models a status message from a low-level robot controller Status includes the position and orientation (Poses) that are the subject of CRCL commands. If any joint status reporting is done, it is assumed that the system sending canonical commands and the system executing them both know the kinematics of the robot and have the same numbering system for the joints, starting with 1. The two systems also have the same understanding of where the zero point is and which direction is positive for each joint. Status items for joints must be configured using a CRCl ConfigureJointReports command. For each joint for which anything is to be reported, ConfigureJointReports specifies:
 *whether joint position should be reported
 *whether joint torque or force should be reported
 *whether joint velocity should be reported
During a CRCL session, until a ConfigureJointReports command has been executed that sets the reporting status for a joint, default joint status is reported for that joint. The ConfigureJointReports command may be used more than once during a session to change joint status reporting.

Java CRLC Simulator – is a graphical user interface (GUI) simulator that accepts joint, Cartesian and other CRCL commands and then simulates robot action. Clients that desire robot status can use the XML interface to communicate and get simulated results as would be expected from an actual robot controller. See Appendix I Java CRCL Simulator.
XMLReader – C++ class to handle all socket communication with the CRCL simulator. Relies on Boost Asio for low level IO handling. The class XMLReader handles the intracies of connecting, initiating, reading, buffering and queing messages within the Boost Asio framework.
Boost.Asio – Boost.Asio is a cross-platform C++ library that was used for network I/O programming because it abstracts the low level socket handling functionality [13]. Boost Asio is used for communication over sockets to the CRCL controller. Boost Asio is very robust with examples and issue solutions pervasively found on the Internet. 
CRCL interface – relied on CodeSynthesis and the Xerces XML DOM parser. Xerces was used to parse the CRCL XML. CodeSynthesis provided support for translating XML into a C++ representation. Given the C++, it was then simple matter to translate the intent of CRCL status message into a corresponding MTConnect intent. Code Synthesis "XSD" tool was used to generate the corresponding C++ classes from the CRCL XSD.


##CRCL Socket Communication
CRCL communicates commands and status using XML. There is no framing mechanism – such as a trailing zero (which is useful since it is illegal in XML) – so the end of a CRCL XML message relies on the detection of a matching XML tag. This is universal to all CRCL communication, message are framed by a pair of opening/closing XML tags. For CRCL status, this means detection of <CRCLStatus> and </CRCLStatus> in the XML message is required to understand when the CRCL status message has completed. This section will further discuss these challenges to reading CRCL XML messages.
As discussed, there is also no terminating character (such as zero) in a CRCL message. Also, CRCL messages can also be of different buffer sizes. So framing the message requires buffering each message, such that the end of a status message is detected with a closing XML tag, and this message may be divided with some of the buffer belonging to the previous or next message. Since there is no CRCL message termination condition, a deadline timer was used to stop asynchronous reading and cancel the read since the last write of the message need not satisfy an asynchronous condition - such as buffer full or matching character. Likewise, often two CRCL will be combined into one aynchronous read operation, so that these two message must be separated by the CRCL streaming reader. 
The method to retrieve CRCL status from a robot controller is to first send a CRCL GetStatus command (that could be combined with other command to configure the status response, but is out of scope) as shown in the (a) portion of Figure 4. Upon receipt of a status command, then the CRCL controller responds with the XML status message as shown in the (b) portion of Figure 4. 
/
_Figure 4 Status Communication to CRCL Controller_
We will assume that the MTConnect has connected to the CRCL simulator socket. Figure 5 shows the components involved in communication.The XMLClient class initiates communication using Boost Asio to first send a "GetStatus" command to the CRCL robot simulator. Using Boost Asio, the class XMLClient then asynchronously reads the reponse from the CRCL robot simulator. When the XMLClient has read a complete message it queues this message onto the message queue. Since the message queue is a shared resource, and multiple threads share this resource, a mutex is used to lock the contents for one thread at a time access. The latest message is retrieved and then the CRCL data expressed in XML can be reinterpreted before storing into the appropriate MTConnect data manager – either device data streams or adapter updates. The translation uses the CRCL interface to parse the CRCL XML message and translate into C++. Once in C++, it is trivial exercise to reinterpret CRCL status into MTConnect lexicon, e.g., mapping enumerations.![Figure1](./images/image1.jpg?raw=true)
_Figure 5 Communication Sequence with Robot CRCL Simulator_
The message queue handles potential timing differences between the XMLClient interface to the CRCL simulator and the MTConnect streaming interface – either Agent Web server or Adapter stream.  At this time, there is only one type of status message, but in the future there may be numerous types of status messages. Thus, for the time being, the latest message on the queue is read, and then the message queue is cleared. 
A major portion of the XMLClient was the handling of the socket stream interface to CRCL simulator. The Boost Asio library was used. Boost Asio uses the Proactor design pattern for event handling in which long running activities are running in an asynchronous part [14]. The major complaint with Boost Asio is that it is more difficult to due to the separation in time and space between operation initiation and completion. Applications may also be harder to debug due to the inverted flow of control and difficulty in understanding when a problem occurred. In the Proactor model, a completion handler is called after the asynchronous event has triggered. In this software pattern, the connect asynchronous event is registered in Boost Asio, and when the connect event happens a "callback" handler method is invoked. Within this connection "callback" handler, the asynchronous read event is setup. This asynchronous read event setup function is used to specify how a stream will be asynchronously read a certain number of bytes of data from a stream. The asynchronous setup itself returns immediately. But, the asynchronous event detection will continue until one of the following conditions is true:
 *The supplied buffer is full. That is, the bytes transferred is equal to the sum of the buffer size.
 *An error occurred. Errors include socket disconnection, or asynchronous read cancelation by the deadline timer. Disconnects cause a discontinuation of reading and the asynchronous connection trigger is then restarted.
Then the asynchronous read event handler will be called. Within the asynchronous read event handler, the asynchronous read event setup must be done again, or no more bytes will be read from the stream. Of note, often the condition of completely filling the buffer is impossible as receiving the exact amount of buffer size is impractical. Because of this, deadline timers are incorporated into Boost Asio to terminate asynchronous reads even if the triggering event (full buffer) has not occurred. So a deadline timer is used to cancel a socket asynchronous read when it expires.
Boost Asio provides io_service, which is a singleton class for servicing I/O. Every program based on Boost.Asio uses an object of type io_service. There is one Asio "io_service" per application program and from our effort it helps if all the asynchronous operations are run in the same thread as this io_service object. Each asynchronous call in Boost Asio is enabled by the io_service methods run, run_one or poll. After all the asynchronous operations were placed in the same thread, the io_service communication responded better. However, Boost Asio was too efficient so an io_service run_one method was combined with a sleep and yield to allow other threads to run and to slow down the Boost Asio operation. 
Figure 6 shows the call sequence involved with the CRCL status reading. Of note, are the synchronous write to send the "Init" during the connection handling and the "GetStatus" command messages sent at the start of every new read of the CRCL simulator status. These are the only Boost Asio operations that were synchronous. Overall, the Boost Asio asynchronous connect operation could wait indefinitely upon startup of the CRCL simulator listener. This is as intended. It is unclear how often Boost Asio tests the socket for the CRCL listener. Upon connection of the socket, the handler for the async connect event is called and it calls the async read and async periodic timer (2 seconds). Either 1) the socket stream is read and the periodic timer is canceled, or 2) the periodic timer expires and the socket async read is canceled, which calls the read handler to see if it has read any bytes or is just waiting for a termination condition. If bytes have been read, there are buffered. In either case, the asynchronous read is called again.![Figure2](./images/image2.jpg?raw=true)
_Figure 6 Asio Communication Sequence_
##CRCL Communication Code Review
The CRCL uses a two thread model 1) one thread to handle communication with CRCL robot and 2) the other thread to handle the ROS interface. Figure 9 shows the main thread spawns thread 2, which handles the ROS message streaming. It was found that if all the Boost Asio operation were not in the same thread, unpredictable results occurred. Unfortunately, it is very difficult to debug problems in Boost Asio, as most of the operation is hidden in a thread, and when no events occur, there is nothing to debug.
Figure 9 shows the code that run in the two threads. Thread1 spawns the MTConnect thread that runs. It is passed argc and argv, which must have a debug or run as a command line argument if the MTConnect executable is to run as an application. Then, thread1 setups up all the Boost Asio event callbacks, and then loop running Boost Asio io_service to handle all the asynchronous events. This framework forms the basis of the MTConnect-CRCL code, which will be described herein.![Figure3](./images/image3.jpg?raw=true)
_Figure 9 Threads – Boost Asio and MTConnect_
Queuing of messages helps manage communication, as the potential to receive parts of the CRCL status at a time is distinctly possible. Since the message queue is a shared resource, and multiple threads share this resource, a mutex is used to lock the contents so that only one thread at a time has access. A Boost Mutex is used. Below is the simple interface to the CRCL status XML message queue.
	typedef std::deque<std::string> xml_message_queue;
	class XMLClient
	{
		void				AddMsgQueue(std::string msg);
		void				ClearMsgQueue();
		std::string			LatestMsgQueue();
		size_t				SizeMsgQueue();
	…
		xml_message_queue xml_msgs;
	…
	};
Upon receiving a message (by detecting the </CRCLStatus> tag in the socket stream), a new message is added with the AddMsgQueue method. The LatestMsgQueue entry returns the latest message (since messages are all the same, the latest is the most accurate description of the current status.) Currently, after retrieving the LatestMsgQueue message, the queue is cleared. 
After a message is retrieved, the data must be reinterpreted before storing into the appropriate MTConnect data manager – either device data streams or adapter updates. The translation uses the CRCL interface to parse the CRCL XML message and translate into C++. Once in C++, it is trivial exercise to reinterpret CRCL status into MTConnect lexicon, e.g., mapping enumerations. For the MTConnect Adapter, the code to perform this decoding is in the routine "gatherDeviceData." If no message, the routine returns, otherwise it call decode(msg) with the current message. 
	void CrclAdapter::gatherDeviceData() {
	    try {
	        // If nothing yet, give up
	        if (xmlClient.SizeMsgQueue() < 1) {
	            return;
	        }
	        std::string msg = xmlClient.LatestMsgQueue();
	        decode(msg);
	    } catch (std::exception e) {
	    } catch (...) {
	    }
	}
The Decode routine determines if the message is in fact a status message, by searching for the string "</CRCLStatus>". If it finds this tag, it uses the class CrclInterface to parse the XML contained in the string, and then converts CrclInterface values into an MTConnect values, and then uses Adapter routine setValue() to update the value. MTConnect Agent and Adapter code is smart enough to know when tags values have been updated, and reacts accordingly to data value changes.
	void CrclAdapter::decode(std::string msg) {
	    if (xmlClient.FindLeadingElement(msg) == "</CRCLStatus>") {
	        CrclInterface crcl;
	        crcl.ParseCRCLStatusString(msg);
	        if (crcl._status.CommandState == "Done")
	            mExecution.setValue(Execution::eREADY);
	        else if (crcl._status.CommandState == "Working")
	            mExecution.setValue(Execution::eACTIVE);
	        else if (crcl._status.CommandState == "Error")
	            mExecution.setValue(Execution::eREADY);
	        mMode.setValue(ControllerMode::eMANUAL);
	        mAlarm.setValue("");
	        mProgram.setValue("");
	        mPosition.setValue(crcl._status.Point[0], crcl._status.Point[1], crcl._status.Point[2]);
	        mXorient.setValue(crcl._status.XAxis[0], crcl._status.XAxis[1], crcl._status.XAxis[2]);
	        mZorient.setValue(crcl._status.ZAxis[0], crcl._status.ZAxis[1], crcl._status.ZAxis[2]);
	    };
	}
The Boost Asio thread is responsible for handling the asynchronous reads of CRCL status messages. The first step, is setting up the asynchronous connect event. This is done by calling the XMLClient::Connect() routine. The connect routine uses a previously stored Internet Protocol (IP) address and socket number, for which it now uses as an endpoint in which to connect. The IP can be a name or a physical address.
	void XMLClient::Connect() {
	    tcp::resolver resolver(*_io_service);
	    tcp::resolver::query query(_ipv4.c_str(), _port.c_str());
	    tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
	    _socket->async_connect(*endpoint_iterator, boost::bind(&XMLClient::HandleConnect,
	            this,
	            boost::asio::placeholders::error));
	}
Upon detection of a socket connect, the routine XMLClient::HandleConnect is called. This routine checks for any errors, if the error has been determined to be connection refused, the Connect routine is called again. There must always be asynchronous events for Boost Asio to detect and handle, or it will terminate. Assuming the socket has been successfully connected, the StartAyncRead()  routine will be called for Boost Asio to detect an asynchronous read event and call the handle as defined in StartAyncRead().  In order for the Java CRCL simulator to operate properly, a CRCL "Init" command must be send to it.
	void XMLClient::HandleConnect(const boost::system::error_code& error) {
	    if (!_socket->is_open()) {
	        Disconnect();
	        return;
	    }
	
	    if (error == boost::asio::error::connection_refused) {
	        Globals::Sleep(_nMSServerConnectRetry);
	        Connect();
	        return;
	    } else if (error == boost::asio::error::already_started) {
	     // not really an error?
	    }
	    // On error, return early.
	    else if (error) {
	        Disconnect();
	        return;  // giving up
	    }
	    StartAyncRead();
	    SyncWrite(CrclInterface().CRCLInitCanonCmd());
	}
The StartAyncRead routine handles the setting up the asynchronous read event for Boost Asio to the complete filling of the read buffer provided to boost::asio::async_read. Because the conditional event of complete filing of the read buffer may not happen (only if the each status message requires multiple reads can the buffer be assured to be full, and the case where the boundary of the read buffer and the status CRCL read are of the same length is not a robust solution), a deadline periodic timer is setup with TimerReset() to time out the read after a certain amount of time and will cancel the asynchronous read. The routine boost::asio::async_read is called with the TCP socket, the read buffer, the condition, and then a callback method is provided using the Boost bind mechanism (which calls this class XMLClient method bytesToRead with parameters for error and number of read bytes transferred by Boost Asio.)
	void XMLClient::StartAyncRead() {
	    try {
	        Globals::DebugMessage("StartAyncRead\n");
	        if (!_socket->is_open()) {
	            DebugBreak();
	        }
	        TimerReset();
	        boost::asio::async_read(*_socket,
	                boost::asio::buffer(data_, max_length), 
	                boost::asio::transfer_at_least(1),
	                boost::bind(&XMLClient::bytesToRead,
	                this,
	                boost::asio::placeholders::error,
	                boost::asio::placeholders::bytes_transferred));
		if(_bSendGetStatus)
		{
			std::string statcmd = CrclInterface().CRCLGetStatusCmd();
		SyncWrite(statcmd);
			_bSendGetStatus=false;
		}
	    } catch (boost::exception & ex) {
	        std::cerr << boost::diagnostic_information(ex);
	       Disconnect();
	    } catch (...) {
	       Disconnect();
	    }
	}
XMLClient::TimerReset()  sets i[ a deadline time to implement the timeout after 2000 milliseconds or 2 seconds. Again, a callback method is provided using the Boost bind mechanism (which calls this class XMLClient method wait_callback with parameters for error and the socket that timed out.
	void XMLClient::TimerReset() {
	   boost::system::error_code ec;
	    int n = _timer->expires_from_now(boost::posix_time::milliseconds(2000), ec);
	    _timer->async_wait(boost::bind(&XMLClient::wait_callback, this, _1, _socket));
	}
The periodic timer event handler checks to see if the timer expired, a general error occurred, or the asynchronous read handler canceled this timer event. 
	void XMLClient::wait_callback(const boost::system::error_code& error, socket_ptr _socket) {
	    if (error == boost::asio::error::operation_aborted) {
	        return; // canceled by asynchronous read event handling
	    } else if (error) {
	        // Unclear why here – maybe disconnect?
	        std::cout << "read_timeout Error - " << error.message() << std::endl;
	        return;
	    }
	    _socket->cancel(); // will cause read_callback to fire with an timeout error
	}
When event signifying that the Java CRCL status socket has finished reading, either thru fulfilling the buffer condition or the periodic timer canceling the socket read, then the routine  bytesToRead() is called. This routine  first cancels the periodic timer.  If there was an error, the socket is Disconnected, which will cause Boost Asio to start listening for a new connection event.  Next the read buffer is appended to the current string buffer. If no end tag has been determined so far, the  FindLeadingElement(_current) routine with the current buffer (includes all read) is called. Finally, the BufferHandler()  is called to determine if a complete message has been read (by looking for the end tag "</CRCLStatus>"). Unless an error has occurred that is not the deadline timer, the StartAyncRead() is called again to insure that Boost Asio continues reading.
	size_t XMLClient::bytesToRead(const error_code& error, size_t bytes_read) {
	    size_t result = bytes_read;
	    _timer->cancel();  // cancel periodic timer
	    if (error == boost::asio::error::eof || boost::asio::error::connection_reset == error) {
	        Disconnect();
	        return 0;
	    } else if (error == boost::asio::error::operation_aborted) {
	    } else if (error) {
	        return 0;
	    }
	
	    if (bytes_read > 0) {
	        AppendBuffer(std::string(data_, data_ + bytes_read));
	        if (_endtag == NonsenseTag()) {
	            _endtag = FindLeadingElement(_current);
	        }
	        BufferHandler(_endtag);
	    }
	    StartAyncRead();
	    return result;
	}

The routine does detection of a complete status message. It searches the buffer string for the ending tag (which is not a dummy tag if a starting tag has been detected.). If it find the ending tag, it divides the message, appending the completed status message onto the queue, and then saving the remaining part of status message for the next read. It is at this point, the end tag is reset to a dummy value, and the signal to send a GetStatus to the CRCL simulator is set.
	bool XMLClient::BufferHandler(std::string endtag) {
	    std::size_t found;
	    if ((found = _current.find(endtag)) != std::string::npos) {
	        found = found + endtag.size();
	        _next = _current.substr(found);
	        _current = _current.substr(0, found);
	        AddMsgQueue(_current);
	        _current.clear();
	        TagReset();
	        _bSendGetStatus=true;
	        return true;
	    }
	    return false;
	}
The routine FindLeadingElement(std::string xml)  is being documented only because it uses Boost regex. Given an XML string, this routine will find the matching element tag after the "<?xml"  header. If found, it returns the ending tag that matches this starting tag, otherwise it return a blank string.
	std::string XMLClient::FindLeadingElement(std::string xml) {
	
	    boost::match_results<std::string::const_iterator> matchResult;
	    bool found;
	    boost::regex e("<[A-Za-z0-9_]+");
	    found = boost::regex_search(xml, matchResult, e);
	    if (found) {
	        std::string elem(matchResult[0]);
	        elem.insert(1, 1, '/');
	        elem = Globals::Trim(elem);
	        elem.append(">"); // not space
	        return elem;
	    }
	    return NonsenseTag();
	}

#Installing Prerequisites
##Installing Xerces c with Ubuntu
https://www.daniweb.com/hardware-and-software/linux-and-unix/threads/409769/ubuntu-11-10-xerces-c As far as I'm aware libxerces is the same as pretty much any other library in Debian based systems. It should be available in the repositories (the exact version will depend on which version of Ubuntu you're running).
You can use apt-get to install the packages for the library and the dev files. Then to use them in your C/C++ programs you simply #include the appropriate headers and link with the library when compiling/linking.
	sudo apt-get update
	apt-cache search libxerces
	sudo apt-get install libxerces-c3.1 libxerces-c-dev
Need include file path CMakeLists.txt:
	include_directories(/usr/include/xercesc)
Link library in CMakeLists.txt:
	link_directories(/usr/lib/x86_64-linux-gnu/)
Need to link against libxerces.a in CMakeLists.txt:
	target_link_libraries(nist_fanuc 
	libxerces-c.a  
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	)
##Installing CodeSynthesis XSD
http://www.codesynthesis.com/products/xsd/download.xhtml 1. Chose the linux deb install file that matches your computer (below 64 bit amd). 2. Download xsd4_.0.0-1amd6_ .deb and it will say open with Ubuntu Software Center 3. Click to install, authenticate and add /usr/include/xsd/cxx/xml as include path.
Need include file path in CMakeLists.txt:
	include_directories(/usr/include/xsd/cxx/xml)
If you cannot run Ubuntu software centerto install CodeSynthesis, you can download the source and install it. You need to go to the web page: http://www.codesynthesis.com/products/xsd/download.xhtml and select:
	xsd-4.0.0-x86_64-linux-gnu.tar.bz2
It will be saved into /usr/local/downloads, but you can save it anywhere. Then cd to where you saved it, and do this:
	tar --bzip2 -xvf xsd-4.0.0-x86_64-linux-gnu.tar.bz2 (dash-dash bzip2, dash-xvf)
It will create a directory xsd-4.0.0-x86_64-linux-gnu.
Make a symbolic link:
	ln -s <path/to/xsd-4.0.0-x86_64-linux-gnu/libxsd/xsd /usr/local/include/xsd
e.g., ln -s /usr/local/xsd-4.0.0-x86_64-linux-gnu/libxsd/xsd /usr/local/include/xsd
##Install Java 1.08 from Oracle for Java CRCL Tool
To build java crcl tool one needs:
	JDK 1.8+ (http://www.oracle.com/technetwork/java/javase/downloads/index.html) and
	maven 3.0.5+ (https://maven.apache.org/download.cgi)
Install maven: $ sudo apt-get install maven
Use the command:
	mvn package
If you see this message at the beginning, bummer:
	Warning: JAVA_HOME environment variable is not set. 
You can check /usr/lib/jvm to see if a 1.8 Java Virtual Machine has been installed. If so, skip the installation step.
So you do not have Java installed. These are instructions for the less than sudo installers. Note, you need the Oracle Java JDK 1.8 version, not the 1.7 version of Ubuntu!!!
Download, unzip and copy to /usr/local/jdk1.8.0_77 or whatever is the latest 1.8 version.
Change you will need to change .bashrc to set the PATH to know where the jdk is installed:
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
Then download crcl4java: https://github.com/wshackle/crcl4java by following directions in Readme.md
