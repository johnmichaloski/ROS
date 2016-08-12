How does the CrclSession work?
----------------------------------
in the main cpp program we declar the controller session thread (with default cycle time).

    // This thread handles new XML messages received from  crcl asio socket.
    RCS::ControllerSession session(DEFAULT_LOOP_CYCLE);
    session.Start(); // start the thread

It uses the Thread template to run cyclically. 

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

       void ControllerSession::Init() {
            std::string info;
           crclinterface= boost::shared_ptr<Crcl::CrclDelegateInterface>(
                   new Crcl::CrclDelegateInterface() );
            _kinematics =  boost::shared_ptr<IKinematics>(new DummyKinematics());
            std::string sStatus = DumpHeader(",") + "\n";
            CsvLogging.Timestamping() = false;
            CsvLogging.LogMessage("Timestamp," + sStatus);
        }


How to publish joints to ROS
----------------------------
http://answers.ros.org/question/43157/trying-to-use-get-joint-state-with-my-urdf/

Using rostopic to send commands to rviz
--------------------------------------

    rostopic pub <topic-name> <topic-type> [data...]

    rostopic pub -1 /joint_states sensor_msgs/JointState '{header: auto, name: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ], velocity: [], effort: []}'

    rostopic pub -1 /joint_states sensor_msgs/JointState '{header: auto, name: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'], position: [0.097, 0.007, -0.590, -0.172, 0.604, -0.142 ], velocity: [], effort: []}'

    rostopic pub /joint_states sensor_msgs/JointState '{header: auto, name: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ], velocity: [], effort: []}'


Writing joint values to rviz and then moving robot and "waiting" until rviz reaches goal