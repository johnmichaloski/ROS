
Fanuc lrmate200id Descarte Demo
================================
ROS-I Packages that must either be installed or clone from github:
fanuc
industrial-core
fanuc-experimental
descartes

All are available as gihub source packages, but will take much longer to compile. So fanu and industrial-core are better installed as ROS packages.


Using apt-get to install necessary ros industrial packages in ROS indigo:

	sudo apt-get install ros-indigo-fanuc
	sudo apt-get install ros-indigo-simple-message
	sudo apt-get install ros-indigo-industrial-core



Setting up ROS with Descartes
-----------------------------
Copy repositories from https://github.com/ros-industrial:
fanuc - NOW A ROS INDIGO PACKAGE  (ros-indigo-fanuc)
fanuc experimental
motoman - CAN OMIT
ros industrial core - NOW A ROS INDIGO PACKAGE

Repositories from ROS-I Consortium https://github.com/ros-industrial-consortium github site:
descartes
descartes_tutorials

Assume catkin_ws has been setup)

	cd /usr/local/michalos/github/ros-industrial
	git clone  https://github.com/ros-industrial/fanuc_experimental.git

	cd /usr/local/michalos/nistfanuc_ws/src
	ln -s /usr/local/michalos/github/ros-industrial-consortium/descartes descartes
	ln -s /usr/local/michalos/github/ros-industrial/fanuc_experimental  fanuc_experimental



How to build just one package using catkin_make?
-----------------------------
A: catkin_make --pkg <my_package_name>

so 

    `catkin_make --pkg nist_fanuc


How do I find the list of available ROS packages?
-----------------------------------------------------
	apt-cache search ros | grep ros-indigo


How do I find the list of installed ROS packages?
--------------------------------------------------------
From ROS FAQ http://wiki.ros.org/FAQ:
On your running system, you can use rospack:

rospack list-names


Is there a way to enable c++11 support for catkin packages?
-----------------------------

    http://catkin-tools.readthedocs.org/en/latest/cheat_sheet.html (kinda wrong)

    set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
    set(CMAKE_CXX_FLAGS "-Wwrite-strings ${CMAKE_CXX_FLAGS}")

how to resolve g++ warning with -std=c++11: ‘auto_ptr’ is deprecated [duplicate]
    https://gcc.gnu.org/bugzilla/show_bug.cgi?id=59325

Creating ros workspace
----------------------
http://wiki.ros.org/catkin/Tutorials/create_a_workspace

	source /opt/ros/indigo/setup.bash
	mkdir -p ~/catkin_ws/src
	cd ~/catkin_ws/src
	catkin_init_workspace
	cd ..
	catkin_make -DCMAKE_BUILD_TYPE=Debug &> log.log

Compiling Fanuc Demo with Debug Information
-----------------------------

    cd ~/catkin_ws
    catkin_make -DCMAKE_BUILD_TYPE=Debug &> log.log
    more log.log
roslaunch fanuc_lrmate200id_moveit_config  moveit_planning_execution.launch  sim:=true

Using IDE to Debug
-----------------------------
After Compiling Fanuc Demo with Debug Information,
Use netbeans to create binary C++ project and read binary file to debug
(found at /home/michalos/catkin_ws/devel/lib/nist_fanuc/nist_fanuc)


Compiling Fanuc Demo with Debug and Error Information Redirected to log file
-----------------------------

    catkin_make -DCMAKE_BUILD_TYPE=Debug &> log.log

-DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=../install -DCATKIN_DEVEL_PREFIX=../devel


Location of exe directory to save ini and other runtime files
-----------------------------
    path	"/home/michalos/catkin_ws/devel/lib/nist_fanuc/nist_fanuc"	

Good websites
----------------------
`http://www.andrew.cmu.edu/user/cgliu/howto.html`



Adding urdfdom and independent package 
-----------------------------
    catkin_make_isolated

 
Installing Xerces c with Ubuntu
-----------------------------
    https://www.daniweb.com/hardware-and-software/linux-and-unix/threads/409769/ubuntu-11-10-xerces-c 
As far as I'm aware libxerces is the same as pretty much any other library in Debian based systems. It should be available in the repositories (the exact version will depend on which version of Ubuntu you're running).

You can use apt-get to install the packages for the library and the dev files.
Then to use them in your C/C++ programs you simply #include the appropriate headers and link with the library when compiling/linking.

    sudo apt-get update
    apt-cache search libxerces
    sudo apt-get install libxerces-c3.1 libxerces-c-dev

Need include file path CMakeLists.txt:
    include_directories(/usr/include/xercesc)

Link library in  CMakeLists.txt:
    link_directories(/usr/lib/x86_64-linux-gnu/)

Need to link against libxerces.a in CMakeLists.txt:
    link_directories(/usr/lib/x86_64-linux-gnu/)

Installing CodeSynthesis XSD
---------------------------------------
http://www.codesynthesis.com/products/xsd/download.xhtml
1) Chose the linux deb install file that matches you computer (below 64 bit amd).
2) Download xsd_4.0.0-1_amd64.deb and it will say open with Ubuntu Software Center
3) Click to install, authenticate and add /usr/include/xsd/cxx/xml as include path.

Need include file path in CMakeLists.txt:
    include_directories(/usr/include/xsd/cxx/xml)


Running Netbeans without Environment setup properly
---------------------------------------
Unfortunately, you need to source ~/catkin_ws/devel/setup.bash to set up Unix shell enironment
variables properly. Since it was not obvious how to run a bash shell before debugging the executable, 
it was decided a different approach must be used. (running a bash script with gdb was attempted). 
Instead environment variables were hard coded into the nist_fanuc program, with the use of the posix 
function "setenv" to set the environment variables (not perfect, but close enough between addition of new 
ROS packages).  

To make it work, I did
    > env | grep indigo
to find all the related ROS environment variables. 
From http://answers.ros.org/question/123581/rosrun-cannot-find-my-executable/ found:
    catkin_find uses the environment variable CMAKE_PREFIX_PATH to find catkin workspaces. 
These workspaces in turn are used in rosrun. ROS_PACKAGE_PATH is no longer enough.

This works, no claims of robustness.  The following code
was placed at the beginning of the nist_fanuc.cpp to set up the environment variables before 
connecting to the roscore (master). (This assumes that all the basic rviz, robot model, etc. has been
launched.)  In the code:

    static void SetupRos(std::string  envname, std::string envval, int overwrite=1)
    {
            setenv( envname.c_str(),envval.c_str(),1 );
    }

           // Setup up environment for netbeans that allows ros utilities to work...
            SetupRos("ROS_MASTER_URI", "http://localhost:11311");
            SetupRos("ROS_DISTRO", "indigo");
            SetupRos("ROS_ROOT", "/opt/ros/indigo/share/ros");
            SetupRos("ROS_ETC_DIR", "/opt/ros/indigo/etc/ros");
            SetupRos("ROS_PACKAGE_PATH", "/home/michalos/catkin_ws/src:/opt/ros/indigo/share:/opt/ros/indigo/stacks");
            SetupRos("ROS_TEST_RESULTS_DIR", "/home/michalos/catkin_ws/build/test_results");
            SetupRos("ROS_ETC_DIR", "/opt/ros/indigo/etc/ros");
            SetupRos("OSLISP_PACKAGE_DIRECTORIES", "/home/michalos/catkin_ws/devel/share/common-lisp");
            SetupRos("PYTHONPATH", "/home/michalos/catkin_ws/devel/lib/python2.7/dist-packages:/opt/ros/indigo/lib/python2.7/dist-packages");
            SetupRos("CMAKE_PREFIX_PATH", "/home/michalos/catkin_ws/devel:/opt/ros/indigo");
            SetupRos("PATH", "/home/michalos/catkin_ws/devel/bin:/opt/ros/indigo/bin:/home/michalos/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin");
            SetupRos("PKG_CONFIG_PATH", "/home/michalos/catkin_ws/devel/lib/x86_64-linux-gnu/pkgconfig:/opt/ros/indigo/lib/x86_64-linux-gnu/pkgconfig:/home/michalos/catkin_ws/devel/lib/pkgconfig:/opt/ros/indigo/lib/pkgconfig");

This does not improve the performance of Netbeans.(Often hogging up to 750M per executable.) 
I could rant/rave about Java, but won't. And I won't complain about lameness of gdb either.

Running
================================================

     source ~/catkin_ws/devel/setup.bash
     cd catkin_ws
     roslaunch nist_fanuc lrmate200id_sim.launch

Naming problem with Robot links
---------------------------------------

    [ WARN] [1456003348.287916295]: World frame 'base_link' does not match model root frame '/base_link', all poses will be transformed to world frame 'base_link'

Problem with Descartes running in Netbeans, even with ROS env set
---------------------------------------

    [ INFO] [1456005198.420170539]: Loading robot model 'fanuc_lrmate200id'...
    sh: 1: catkin_find: not found
    [ERROR] [1456005198.949779328]: The kinematics plugin (manipulator) failed to load. Error: Could not find library corresponding to plugin fanuc_lrmate200id_manipulator_kinematics/IKFastKinematicsPlugin. Make sure the plugin description XML file has the correct name of the library and that the library actually exists.
    [ERROR] [1456005198.949844984]: Kinematics solver could not be instantiated for joint group manipulator.
    [ INFO] [1456005199.039875870]: Generated 10 random seeds
    Group 'manipulator' using 6 variables
      * Joints:
        'joint_1' (Revolute)
        'joint_2' (Revolute)
        'joint_3' (Revolute)
        'joint_4' (Revolute)
        'joint_5' (Revolute)
        'joint_6' (Revolute)
        'joint_6-tool0' (Fixed)
      * Variables:
        'joint_1', index 0 in full state, index 0 in group state
            P.bounded [-2.965, 2.965]; V.bounded [-7.85, 7.85]; A.bounded [-1.57, 1.57];
        'joint_2', index 1 in full state, index 1 in group state
            P.bounded [-1.74533, 2.53073]; V.bounded [-6.63, 6.63]; A.bounded [-1.326, 1.326];
        'joint_3', index 2 in full state, index 2 in group state
            P.bounded [-2.45097, 4.88692]; V.bounded [-9.08, 9.08]; A.bounded [-1.816, 1.816];
        'joint_4', index 3 in full state, index 3 in group state
            P.bounded [-3.315, 3.315]; V.bounded [-9.6, 9.6]; A.bounded [-1.92, 1.92];
        'joint_5', index 4 in full state, index 4 in group state
            P.bounded [-2.18, 2.18]; V.bounded [-9.51, 9.51]; A.bounded [-1.902, 1.902];
        'joint_6', index 5 in full state, index 5 in group state
            P.bounded [-6.285, 6.285]; V.bounded [-17.45, 17.45]; A.bounded [-3.49, 3.49];
      * Variables Index List:
        0 1 2 3 4 5 (contiguous)

    [ WARN] [1456005199.040243338]: World frame 'base_link' does not match model root frame '/base_link', all poses will be transformed to world frame 'base_link'


Problem with Descartes joint planning
---------------------------------------
unable to calculate edge weight of joint transitions for joint trajectories
No IK either when run from bash terminal



Standalone method to read URDF file without ROS
---------------------------------------
THis is a headache since C++ doesn't allow mutliple definitions of 
the same classes and typedefs, so there will be naming and typing collisions
between the ROS urdf model and the Standalone reader. Useful for testing when
you dont want the pain and overhead of ROS.

    #ifdef STANDALONE
            // The following is an example of how to read a urdf file
            RCS::Controller.wm.robot_model = urdf::parseURDFFile(Globals.ExeDirectory + "lrmate200id.urdf");

            if (!RCS::Controller.wm.robot_model) {
                throw std::runtime_error("ERROR: Model Parsing the xml failed");
            }
            std::string str = urdf::GenerateTable(RCS::Controller.wm.robot_model);
            Globals.WriteFile(Globals.ExeDirectory + "robotmodel.html", str);
    #endif

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





GDB
---------------------------

    b main
    r
    s (step)
    n (next)
    print *(points._M_impl._M_start)@points.size()

    (gdb) break source.cpp:8
    (gdb) run
    (gdb) p vec.begin()
    $1 = {
       _M_current = 0x300340
    }
    (gdb) p $1._M_current->c_str()
    $2 = 0x3002fc "Hello"
    (gdb) p $1._M_current +1
    $3 = (string *) 0x300344
    (gdb) p $3->c_str()
    $4 = 0x30032c "world"

STOPPING POSTGRESQL-9.4
------------------------------
    sudo pkill postgres
    sudo update-rc.d postgresql-9.4 disable

An easy way to create a ubuntu desktop shortcut?
-----------------------------------------------
    Open Nautilus
    Navigate to /usr/share/applications
    Right-click on the application you want to use and select copy
    Click on your desktop and select paste
    Right click on the icon that has just been created and select properties
    On the Permissions tab check Execute then click Close



    http://gazebosim.org/tutorials?tut=drcsim_ros_cmds&cat=drcsim
    RCSInterpreter::ParseCommand
    RobotStatus::Action

    PID: http://wiki.ros.org/pr2_mechanism/Tutorials/Adding%20a%20PID%20to%20a%20realtime%20joint%20controller
    Joint XML https://github.com/RethinkRobotics/baxter_simulator/blob/master/baxter_sim_controllers/src/baxter_velocity_controller.cpp

http://gazebosim.org/tutorials?tut=drcsim_ros_cmds&cat=drcsim 


How to publish joints to ROS
----------------------------
http://answers.ros.org/question/43157/trying-to-use-get-joint-state-with-my-urdf/



How to visualize trajectory path in moveit
----------------------------------------------------
So I think I have a workable answer to my own question.  Using the
Python API, the following code snippet appears to alter the trajectory
speed as desired (here the speed is doubled):

         traj = right_arm.plan()
         new_traj = RobotTrajectory()
         new_traj.joint_trajectory = traj.joint_trajectory
         n_joints = len(traj.joint_trajectory.joint_names)
         n_points = len(traj.joint_trajectory.points)

         spd = 2.0

         for i in range(n_points):
             traj.joint_trajectory.points[i].time_from_start =
    traj.joint_trajectory.points[i].time_from_start / spd
             for j in range(n_joints):
    new_traj.joint_trajectory.points[i].velocities[j] =
    traj.joint_trajectory.points[i].velocities[j] * spd
    new_traj.joint_trajectory.points[i].accelerations[j] =
    traj.joint_trajectory.points[i].accelerations[j] * spd
    new_traj.joint_trajectory.points[i].positions[j] =
    traj.joint_trajectory.points[i].positions[j]

         self.right_arm.execute(new_traj) 
    Web references:
    https://github.com/ros-controls/ros_controllers/blob/jade-devel/joint_trajectory_controller/test/joint_trajectory_controller_test.cpp
    https://github.com/nttputus/wp6_manipulator/blob/master/crops_wp6_arm_navigation_tutorials/src/display_trajectory_tutorial.cpp


How the fanuc lrmate 200id IKFast kinematics plug in is installed
------------------------------------------
    http://sdk.rethinkrobotics.com/wiki/MoveIt_Tutorial

    rosed <myrobot_name>_moveit_config/config/kinematics.yaml

Edit these parts:

    <planning_group_name>:
      kinematics_solver: <moveit_ik_plugin_pkg>/IKFastKinematicsPlugin

-OR-

      kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin

IBID: /home/michalos/catkin_ws/src/fanuc_experimental/fanuc_lrmate200ib3l_moveit_config/config/kinematics.yaml
    manipulator:

      kinematics_solver: fanuc_lrmate200id_manipulator_kinematics/IKFastKinematicsPlugin
      kinematics_solver_attempts: 3
      kinematics_solver_search_resolution: 0.005
      kinematics_solver_timeout: 0.005

You will now need to recompile and install the IKFast plugins.

       catkin_make
       catkin_make install --pkg fanuc_lrmate200ib3l_manipulator_kinematics_plugin



    michalos@rufous:~/catkin_ws$ rospack list
    ....
    fanuc_lrmate200id_moveit_config /home/michalos/catkin_ws/src/fanuc_experimental/fanuc_lrmate200id_moveit_config
    fanuc_lrmate200id_moveit_plugins /home/michalos/catkin_ws/src/fanuc_experimental/fanuc_lrmate200id_moveit_plugins
    fanuc_lrmate200id_support /home/michalos/catkin_ws/src/fanuc_experimental/fanuc_lrmate200id_support
...


installing uncrustify - C++ code formatter
-----------------------------------------

    sudo apt-get install uncrustify
    sudo apt-get install universalindentgui

    michalos@rufous:~/catkin_ws$ universalindentgui


Diplay contents of rostopic to see current robot pose
--------------------------------------------------------
    rostopic echo  /move_group/goal

    header: 
      seq: 30
      stamp: 
        secs: 1456420731
        nsecs: 958832629
      frame_id: ''
    goal_id: 
      stamp: 
        secs: 1456420731
        nsecs: 958833341
      id: /rviz_rufous_6391_9202004766824814528-31-1456420731.958833341
    goal: 
      request: 
        workspace_parameters: 
          header: 
            seq: 0
            stamp: 
              secs: 1456420731
              nsecs: 958705952
            frame_id: /base_link
          min_corner: 
            x: -1.0
            y: -1.0
            z: -1.0
          max_corner: 
            x: 1.0
            y: 1.0
            z: 1.0
        start_state: 
          joint_state: 
            header: 
              seq: 0
              stamp: 
                secs: 0
                nsecs: 0
              frame_id: /base_link
            name: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
            position: [-0.9951596824786128, -1.161361427138559, 0.6456418463565006, 3.1368453900839213, 2.0662861177525222, -1.4036373909368032]
            velocity: []
            effort: []
          multi_dof_joint_state: 
            header: 
              seq: 0
              stamp: 
                secs: 0
                nsecs: 0
              frame_id: /base_link
            joint_names: []
            transforms: []
            twist: []
            wrench: []
          attached_collision_objects: []
          is_diff: False
        goal_constraints: 
          - 
            name: ''
            joint_constraints: 
              - 
                joint_name: joint_1
                position: 1.75594190178
                tolerance_above: 0.0001
                tolerance_below: 0.0001
                weight: 1.0
              - 
                joint_name: joint_2
                position: -1.50958199864
                tolerance_above: 0.0001
                tolerance_below: 0.0001
                weight: 1.0
              - 
                joint_name: joint_3
                position: 2.60972229461
                tolerance_above: 0.0001
                tolerance_below: 0.0001
                weight: 1.0
              - 
                joint_name: joint_4
                position: -1.55001527988
                tolerance_above: 0.0001
                tolerance_below: 0.0001
                weight: 1.0
              - 
                joint_name: joint_5
                position: 1.68989821207
                tolerance_above: 0.0001
                tolerance_below: 0.0001
                weight: 1.0
              - 
                joint_name: joint_6
                position: 2.29309630865
                tolerance_above: 0.0001
                tolerance_below: 0.0001
                weight: 1.0
            position_constraints: []
            orientation_constraints: []
            visibility_constraints: []
        path_constraints: 
          name: ''
          joint_constraints: []
          position_constraints: []
          orientation_constraints: []
          visibility_constraints: []
        trajectory_constraints: 
          constraints: []
        planner_id: ''
        group_name: manipulator
        num_planning_attempts: 1
        allowed_planning_time: 1.0
        max_velocity_scaling_factor: 1.0
      planning_options: 
        planning_scene_diff: 
          name: ''
          robot_state: 
            joint_state: 
              header: 
                seq: 0
                stamp: 
                  secs: 0
                  nsecs: 0
                frame_id: ''
              name: []
              position: []
              velocity: []
              effort: []
            multi_dof_joint_state: 
              header: 
                seq: 0
                stamp: 
                  secs: 0
                  nsecs: 0
                frame_id: ''
              joint_names: []
              transforms: []
              twist: []
              wrench: []
            attached_collision_objects: []
            is_diff: True
          robot_model_name: ''
          fixed_frame_transforms: []
          allowed_collision_matrix: 
            entry_names: []
            entry_values: []
            default_entry_names: []
            default_entry_values: []
          link_padding: []
          link_scale: []
          object_colors: []
          world: 
            collision_objects: []
            octomap: 
              header: 
                seq: 0
                stamp: 
                  secs: 0
                  nsecs: 0
                frame_id: ''
              origin: 
                position: 
                  x: 0.0
                  y: 0.0
                  z: 0.0
                orientation: 
                  x: 0.0
                  y: 0.0
                  z: 0.0
                  w: 0.0
              octomap: 
                header: 
                  seq: 0
                  stamp: 
                    secs: 0
                    nsecs: 0
                  frame_id: ''
                binary: False
                id: ''
                resolution: 0.0
                data: []
          is_diff: True
        plan_only: False
        look_around: False
        look_around_attempts: 0
        max_safe_execution_cost: 0.0
        replan: False
        replan_attempts: 0
        replan_delay: 2.0
    ---

Display only the pose of the robot
----------------------------------

    sudo apt-get install ros-indigo-robot-pose-publisher
    rosrun robot_pose_publisher robot_pose_publisher

FAILED!


Moveit failed to plan also
-------------------------

    [ INFO] [1456439525.551846365]: Planning request received for MoveGroup action. Forwarding to planning pipeline.
    [ WARN] [1456439525.554072295]: Orientation constraint for link 'tool0' is probably incorrect: 0.000000, 0.000000, 0.000000, 0.000000. Assuming identity instead.
    [ WARN] [1456439525.554147337]: Orientation constraint for link 'tool0' is probably incorrect: 0.000000, 0.000000, 0.000000, 0.000000. Assuming identity instead.
    [ INFO] [1456439525.554827784]: LBKPIECE1: Starting planning with 1 states already in datastructure
    [ INFO] [1456439526.966671637]: Found a contact between 'link_4' (type 'Robot link') and 'base_link' (type 'Robot link'), which constitutes a collision. Contact information is not stored.
    [ INFO] [1456439526.966721656]: Collision checking is considered complete (collision was found and 0 contacts are stored)
    [ INFO] [1456439526.967459590]: Found a contact between 'link_6' (type 'Robot link') and 'base_link' (type 'Robot link'), which constitutes a collision. Contact information is not stored.
    [ INFO] [1456439526.967502013]: Collision checking is considered complete (collision was found and 0 contacts are stored)
    [ INFO] [1456439526.968228658]: Found a contact between 'link_4' (type 'Robot link') and 'base_link' (type 'Robot link'), which constitutes a collision. Contact information is not stored.
    [ INFO] [1456439526.968269033]: Collision checking is considered complete (collision was found and 0 contacts are stored)
    [ INFO] [1456439526.969014693]: Found a contact between 'link_6' (type 'Robot link') and 'base_link' (type 'Robot link'), which constitutes a collision. Contact information is not stored.
    [ INFO] [1456439526.969054959]: Collision checking is considered complete (collision was found and 0 contacts are stored)
    [ERROR] [1456439528.325962429]: LBKPIECE1: Unable to sample any valid states for goal tree
    [ INFO] [1456439528.325996844]: LBKPIECE1: Created 1 (1 start + 0 goal) states in 1 cells (1 start (1 on boundary) + 0 goal (0 on boundary))
    [ INFO] [1456439528.326030131]: No solution found after 2.771767 seconds
    [ INFO] [1456439528.326055767]: Unable to solve the planning problem



Moveit planner hangs
---------------------

        if (!group->plan(my_plan))
            return false;

You must have multithreaded enabled before any moveit planning.

            // Required for multithreaded communication with moveit components
            ros::AsyncSpinner spinner(1);
            spinner.start();


moveit planning with joints
--------------------------


        joints.position = moveit.SetRandomJoints();
        std::cout << "Random assigned joints=" << VectorDump<double> (joints.position).c_str();
        moveit.Plan(joints);
        sleep(15.0);
        js = moveit.GetJointValues();
        std::cout << "Current joints=" << VectorDump<double> (js);

You must wait for rviz to move. You get all kinds of plans.

rviz not moving
----------------

    pub <topic-name> <topic-type> [data...]

    rostopic pub -1 /joint_states sensor_msgs/JointState '{header: auto, name: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ], velocity: [], effort: []}'

    rostopic pub -1 /joint_states sensor_msgs/JointState '{header: auto, name: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'], position: [0.097, 0.007, -0.590, -0.172, 0.604, -0.142 ], velocity: [], effort: []}'

    rostopic pub /joint_states sensor_msgs/JointState '{header: auto, name: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ], velocity: [], effort: []}'


Writing joint values to rviz and then moving robot and "waiting" until rviz reaches goal
--------------------------------------------------------------------------------------------
Fixed ToVector and made template parameter mandatory.
THen added wait until "AtGoal" motion control. Rviz need processing and that
only comes when waiting. Again, make sure  ros::AsyncSpinner is running!

            joint.position=ToVector<double>(6, 0.097, 0.007, -0.590, -0.172, 0.604, -0.142 );
            jointWriter->JointTrajectoryPositionWrite(joint);
            while( !MotionControl::AtGoal(joint, curjoints) && n++ < 100) {
                curjoints = jointReader->GetCurrentReadings();
                std::cout << "Goal joints=" << VectorDump<double> (joint.position).c_str();
                std::cout << "Cur  joints=" << VectorDump<double> (curjoints.position).c_str();
                Globals.Sleep(100); // 100 milliseconds
            }

    template<typename T>
    static bool epsiloncompare (T &i, T& j) {
      return (fabs(i-j) < MotionControl::epsilon);
    }
    template<typename T>
    static bool VectorCompare(std::vector<T> vec1,std::vector<T> vec2 )
    {
       return std::equal (vec1.begin(), vec1.end(),  vec2.begin() , epsiloncompare) ;
    }
    bool MotionControl::AtGoal(JointState goal, JointState current,  double epsilon)
    {
        MoveitTrajectory::epsilon=epsilon;
        return VectorCompare<double>(goal.position, current.position);
    }

Turn off Planned Path visualization in Rviz - confusing
---------------------------------------------------------
In Rviz GUI, navigate to Planned Path, and uncheck "Show Robot Visual"

Not sure how this parameter is set in /move_group/display_planned_path


Using waypoints in moveit
-------------------------

Calculated a waypoint every 1mm 

    std::vector<JointState> MoveitTrajectory::Plan(std::vector<urdf::Pose>& pwaypoints) {
        std::vector<geometry_msgs::Pose> waypoints;
        for(size_t i=0; i< pwaypoints.size(); i++)
        {    
            waypoints.push_back(PoseMsg2UrdfPose(pwaypoints[i]));
        }

        moveit_msgs::RobotTrajectory trajectory;
        double fraction = group->computeCartesianPath(waypoints,
                0.001, // cartesian path to be interpolated at a resolution of 1 mm 
                0.0, // NO jump_threshold
                trajectory);  // trajectory.joint_trajectory.points  (position)
        std::vector<JointState> points;
        for(size_t j=0; j< trajectory.joint_trajectory.points.size(); j++)
        {
            JointState traj;
            traj.position = trajectory.joint_trajectory.points[j].positions;
            points.push_back(traj);
        }
        return points;
    }

Test code:

           MoveitTrajectory moveit(nh);
            MotionControl motioncontrol;
            urdf::Pose goalpose;
            goalpose.position =urdf::Vector3(.28,-0.7,1.0);
            goalpose.rotation.setFromRPY(0.,0.,0.);
            
            int nIncr=motioncontrol.computeIncrements (RCS::Controller.status.currentpose, goalpose);
            std::vector<urdf::Pose> poses = motioncontrol.computeWaypoints(RCS::Controller.status.currentpose, goalpose, nIncr, true );
            std::vector<JointState> points = moveit.Plan(poses);
            for(size_t k=0; k< points.size(); k++)
            {
                std::cout <<  VectorDump<double> (points[k].position);
                jointWriter->JointTrajectoryPositionWrite(points[k]);
            }



Running Fanuc Robot - notes
--------------------------
Powerup:
1. Turn on power on front of controller (keyed)
2. If auto mode, make sure teach pendant upper left corner knob is OFF
3. If in fault- hold deadman switch halfway, Hold [Shift], press [Reset] key
4. reset to local mode
   Menu -> [32] Remote/Local/... [F4] Local [Enter]
5. Start ROS programs
   [Teach][Select] => scroll down to ROS, number 39 hit [Enter]
   (starts 2 programs)
6. Cycle start 
   Green Auto button on front controller panel, press/release, green
   light should go on.

Powerdown:
1. Kill ROS programs - DO TWICE - 2 programs running
   [FCNT] -> 1 -> [ENTER]  
   [FCNT] -> 1 -> [ENTER]  
If fanuc controller faulted, and have to manually reset joint to safe position
1. Turn controller box to teach pendant from auto
2. hold deadman switch half-on, [SHIFT] Hold, hit [Reset]
3. Now move robot - +/- joint key or xyz key
4. Note to increase traversal- feedoverride in green xx% field in upper right corner 

Run ROS Fanuc demo

    roslaunch fanuc_lrmate200id_moveit_config  moveit_planning_execution.launch 
      sim:=false   robot_ip:=129.6.78.111

Run RVIZ roslaunch with Fanuc LRMate 200id 
    #!/bin/bash
    source /home/michalos/catkin_ws/devel/setup.bash
    roslaunch nist_fanuc lrmate200id_sim.launch

    sleep 100

Launch file:

    <?xml version="1.0"?>
    <launch>
      
      <arg name="sim" default="true" />
      
      <include file="$(find fanuc_lrmate200id_moveit_config)/launch/moveit_planning_execution.launch">
        <arg name="sim" value="$(arg sim)"/>
      </include>

    </launch>



Fanuc 200id kinematics plugin

/home/michalos/catkin_ws/src/fanuc_experimental/fanuc_lrmate200id_moveit_config/config/kinematics.yaml

    manipulator:
      kinematics_solver: fanuc_lrmate200id_manipulator_kinematics/IKFastKinematicsPlugin
      kinematics_solver_attempts: 3
      kinematics_solver_search_resolution: 0.005
      kinematics_solver_timeout: 0.005

Maybe:
    # kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
Worked!

    void visualize(ros::NodeHandle nh, moveit_msgs::MotionPlanResponse response) {
	    ROS_INFO("Visualizing the trajectory");
	    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	    moveit_msgs::DisplayTrajectory display_trajectory;
	     
	    display_trajectory.trajectory_start = response.trajectory_start;
	    display_trajectory.trajectory.push_back(response.trajectory);
	    display_publisher.publish(display_trajectory);
    }

Ros Cartesian Planning with assigned plugin

    int main(int argc, char **argv) {
	    ros::init (argc, argv, "planning_pipeline");
	    ros::AsyncSpinner spinner(1);
	    spinner.start();
	    ros::NodeHandle node_handle("~");
	
	    //map<int, Controller*> controllersOrder;
	    //vector<Controller>  controllers ;//= initControllers(node_handle);
	
	     
	    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
	     
	    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
	    planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));
	     
	    //Sleep a little to allow time to startup rviz, etc.
	    ros::WallDuration sleep_time(5.0);
	    sleep_time.sleep();
	     
	    // A tolerance of 0.01 m is specified in position
	    // and 0.01 radians in orientation
	    vector<double> tolerance_pose(3, 0.01);
	    vector<double> tolerance_angle(3, 0.01);
	     
	    // Pose Goal
	    // ^^^^^^^^^
	    // We will now create a motion plan request for the right arm of the PR2
	    // specifying the desired pose of the end-effector as input.
	    planning_interface::MotionPlanRequest req;
	    planning_interface::MotionPlanResponse res;
	    geometry_msgs::PoseStamped pose;
	    pose.header.frame_id = "torso";
	    pose.pose.position.x = -0.000006;
	    pose.pose.position.y = 0.05;
	    pose.pose.position.z = -0.24;
	     
	    req.group_name = "leg_left";
	    req.planner_id = "RRTkConfigDefault";
	    req.allowed_planning_time=5;
	    req.num_planning_attempts = 5;
	     
	    moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("l_sole", pose, tolerance_pose, tolerance_angle);
	    req.goal_constraints.push_back(pose_goal);
	     
	    planning_pipeline->generatePlan(planning_scene, req, res);
	    if(res.error_code_.val != res.error_code_.SUCCESS) {
		    ROS_ERROR("Could not compute plan successfully");
		    return 0;
	    }
	     
	    moveit_msgs::MotionPlanResponse response;
	    res.getMessage(response);
	
	    // Visualize the result
		    ROS_INFO("Visualizing the trajectory");
	    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	    moveit_msgs::DisplayTrajectory display_trajectory;
	     
	    display_trajectory.trajectory_start = response.trajectory_start;
	    display_trajectory.trajectory.push_back(response.trajectory);
	    display_publisher.publish(display_trajectory);
	
	
	    //visualize(node_handle, response);

	    ros::waitForShutdown();
	
	    return 0;


rosparam - get list of ROS parameter from paramserver
------------
Does not provide the values....

    michalos@rufous:~/catkin_ws$ rosparam list
    /controller_joint_names
    /move_group/allow_trajectory_execution
    /move_group/allowed_execution_duration_scaling
    /move_group/allowed_goal_duration_margin
    /move_group/capabilities
    /move_group/controller_list
    /move_group/jiggle_fraction
    /move_group/manipulator/longest_valid_segment_fraction
    /move_group/manipulator/planner_configs
    /move_group/manipulator/projection_evaluator
    /move_group/max_range
    /move_group/max_safe_path_cost
    /move_group/moveit_controller_manager
    /move_group/moveit_manage_controllers
    /move_group/octomap_resolution
    /move_group/ompl/display_random_valid_states
    /move_group/ompl/link_for_exploration_tree
    /move_group/ompl/maximum_waypoint_distance
    /move_group/ompl/minimum_waypoint_count
    /move_group/ompl/simplify_solutions
    /move_group/plan_execution/max_replan_attempts
    /move_group/plan_execution/record_trajectory_state_frequency
    /move_group/planner_configs/BKPIECEkConfigDefault/border_fraction
    /move_group/planner_configs/BKPIECEkConfigDefault/failed_expansion_score_factor
    /move_group/planner_configs/BKPIECEkConfigDefault/min_valid_path_fraction
    /move_group/planner_configs/BKPIECEkConfigDefault/range
    /move_group/planner_configs/BKPIECEkConfigDefault/type
    /move_group/planner_configs/ESTkConfigDefault/goal_bias
    /move_group/planner_configs/ESTkConfigDefault/range
    /move_group/planner_configs/ESTkConfigDefault/type
    /move_group/planner_configs/KPIECEkConfigDefault/border_fraction
    /move_group/planner_configs/KPIECEkConfigDefault/failed_expansion_score_factor
    /move_group/planner_configs/KPIECEkConfigDefault/goal_bias
    /move_group/planner_configs/KPIECEkConfigDefault/min_valid_path_fraction
    /move_group/planner_configs/KPIECEkConfigDefault/range
    /move_group/planner_configs/KPIECEkConfigDefault/type
    /move_group/planner_configs/LBKPIECEkConfigDefault/border_fraction
    /move_group/planner_configs/LBKPIECEkConfigDefault/min_valid_path_fraction
    /move_group/planner_configs/LBKPIECEkConfigDefault/range
    /move_group/planner_configs/LBKPIECEkConfigDefault/type
    /move_group/planner_configs/PRMkConfigDefault/max_nearest_neighbors
    /move_group/planner_configs/PRMkConfigDefault/type
    /move_group/planner_configs/PRMstarkConfigDefault/type
    /move_group/planner_configs/RRTConnectkConfigDefault/range
    /move_group/planner_configs/RRTConnectkConfigDefault/type
    /move_group/planner_configs/RRTkConfigDefault/goal_bias
    /move_group/planner_configs/RRTkConfigDefault/range
    /move_group/planner_configs/RRTkConfigDefault/type
    /move_group/planner_configs/RRTstarkConfigDefault/delay_collision_checking
    /move_group/planner_configs/RRTstarkConfigDefault/goal_bias
    /move_group/planner_configs/RRTstarkConfigDefault/range
    /move_group/planner_configs/RRTstarkConfigDefault/type
    /move_group/planner_configs/SBLkConfigDefault/range
    /move_group/planner_configs/SBLkConfigDefault/type
    /move_group/planner_configs/TRRTkConfigDefault/frountierNodeRatio
    /move_group/planner_configs/TRRTkConfigDefault/frountier_threshold
    /move_group/planner_configs/TRRTkConfigDefault/goal_bias
    /move_group/planner_configs/TRRTkConfigDefault/init_temperature
    /move_group/planner_configs/TRRTkConfigDefault/k_constant
    /move_group/planner_configs/TRRTkConfigDefault/max_states_failed
    /move_group/planner_configs/TRRTkConfigDefault/min_temperature
    /move_group/planner_configs/TRRTkConfigDefault/range
    /move_group/planner_configs/TRRTkConfigDefault/temp_change_factor
    /move_group/planner_configs/TRRTkConfigDefault/type
    /move_group/planning_plugin
    /move_group/planning_scene_monitor/publish_geometry_updates
    /move_group/planning_scene_monitor/publish_planning_scene
    /move_group/planning_scene_monitor/publish_planning_scene_hz
    /move_group/planning_scene_monitor/publish_state_updates
    /move_group/planning_scene_monitor/publish_transforms_updates
    /move_group/request_adapters
    /move_group/sense_for_plan/discard_overlapping_cost_sources
    /move_group/sense_for_plan/max_cost_sources
    /move_group/sense_for_plan/max_look_attempts
    /move_group/sense_for_plan/max_safe_path_cost
    /move_group/start_state_max_bounds_error
    /move_group/trajectory_execution/allowed_execution_duration_scaling
    /move_group/trajectory_execution/execution_duration_monitoring
    /move_group/trajectory_execution/execution_velocity_scaling
    /robot_description
    /robot_description_kinematics/manipulator/kinematics_solver
    /robot_description_kinematics/manipulator/kinematics_solver_attempts
    /robot_description_kinematics/manipulator/kinematics_solver_search_resolution
    /robot_description_kinematics/manipulator/kinematics_solver_timeout
    /robot_description_planning/joint_limits/joint_1/has_acceleration_limits
    /robot_description_planning/joint_limits/joint_1/has_velocity_limits
    /robot_description_planning/joint_limits/joint_1/max_acceleration
    /robot_description_planning/joint_limits/joint_1/max_velocity
    /robot_description_planning/joint_limits/joint_2/has_acceleration_limits
    /robot_description_planning/joint_limits/joint_2/has_velocity_limits
    /robot_description_planning/joint_limits/joint_2/max_acceleration
    /robot_description_planning/joint_limits/joint_2/max_velocity
    /robot_description_planning/joint_limits/joint_3/has_acceleration_limits
    /robot_description_planning/joint_limits/joint_3/has_velocity_limits
    /robot_description_planning/joint_limits/joint_3/max_acceleration
    /robot_description_planning/joint_limits/joint_3/max_velocity
    /robot_description_planning/joint_limits/joint_4/has_acceleration_limits
    /robot_description_planning/joint_limits/joint_4/has_velocity_limits
    /robot_description_planning/joint_limits/joint_4/max_acceleration
    /robot_description_planning/joint_limits/joint_4/max_velocity
    /robot_description_planning/joint_limits/joint_5/has_acceleration_limits
    /robot_description_planning/joint_limits/joint_5/has_velocity_limits
    /robot_description_planning/joint_limits/joint_5/max_acceleration
    /robot_description_planning/joint_limits/joint_5/max_velocity
    /robot_description_planning/joint_limits/joint_6/has_acceleration_limits
    /robot_description_planning/joint_limits/joint_6/has_velocity_limits
    /robot_description_planning/joint_limits/joint_6/max_acceleration
    /robot_description_planning/joint_limits/joint_6/max_velocity
    /robot_description_semantic
    /rosdistro
    /roslaunch/uris/host_rufous__60560
    /rosversion
    /run_id
    /rviz_rufous_23871_3766625969024100662/manipulator/kinematics_solver
    /rviz_rufous_23871_3766625969024100662/manipulator/kinematics_solver_attempts
    /rviz_rufous_23871_3766625969024100662/manipulator/kinematics_solver_search_resolution
    /rviz_rufous_23871_3766625969024100662/manipulator/kinematics_solver_timeout
    /rviz_rufous_23871_3766625969024100662/motionplanning_planning_scene_monitor/publish_geometry_updates
    /rviz_rufous_23871_3766625969024100662/motionplanning_planning_scene_monitor/publish_planning_scene
    /rviz_rufous_23871_3766625969024100662/motionplanning_planning_scene_monitor/publish_planning_scene_hz
    /rviz_rufous_23871_3766625969024100662/motionplanning_planning_scene_monitor/publish_state_updates
    /rviz_rufous_23871_3766625969024100662/motionplanning_planning_scene_monitor/publish_transforms_updates
    michalos@rufous:~/catkin_ws$ 



Remove installed package from ubuntu
--------------------------------------
    sudo apt-get remove grip

Pose conversion tests
-------------------------

Old way:
    CRCL conversion tests
    CRCL Pose 0.4643,0.02436,1.275,0.01676,0.08284,0.9964,0.2896,0.9535,-0.08413,
    Urdf Pose Translation = 464.3:24.36:1275
    Rotation = -95.0423:16.8334:-88.9969

Eigen way: no rpy intermediary step
    CRCL Pose 0.4643,0.02436,1.275,0.01676,0.08284,0.9964,0.2896,0.9535,-0.08413,
    Urdf Pose Translation = 464.3:24.36:1275
    Rotation = 174.572:-85.1698:78.5552

    urdf::Pose Convert(Crcl::PoseType & pose, double lengthConversion) {
            urdf::Pose p;

            p.position.x = pose.Point().X() * lengthConversion;
            p.position.y = pose.Point().Y() * lengthConversion;
            p.position.z = pose.Point().Z() * lengthConversion;

            Eigen::Matrix3d mat=GetEigenRotMatrix(GetVector3D(pose.XAxis()), GetVector3D(pose.ZAxis()));
            Eigen::Quaterniond q(mat);
            p.rotation.x = q.x();
            p.rotation.y = q.y();
            p.rotation.z = q.z();
            p.rotation.w = q.w();
            return p;
        }

Using ROS whose RPY from quaterion doesnt work!
-----------------------------------------------------------
Instead use posemath, gomotion rpy from matrix via quaterion

    GotoCRCL Pose 0.465,0,0.745,-2.051e-10,-8.979e-11,1,1,0,2.051e-10,
    Goto urdf Pose Translation = 465:0:745
    Rotation = 180:-90:0



Position Only IK
------------------
Position only IK can easily be enabled (only if you are using the KDL Kinematics Plugin) by adding the following line to your kinematics.yaml file (for the particular group that you want to solve IK for):

position_only_ik: True

I suppose this is "easily" if you dont ever want to change it dynamically or programmatically without modifying the yaml file....


Problems iwth ikfast
-----------------------
http://answers.ros.org/question/205781/moveit-inverse_kinematics-c-api/ 
Side note: I am aware that FastIK is a possible alternative to KDL, however it requires to install OpenRave, which seems to be problematic under Ubuntu 14.04. Also the ros converter urdf_to_collada for indigo is broken and not working. There is just generally speaking a lot of hassle to get an IK solver at the moment under indigo.

JM: Agree!

Person used KDL IK directly

Markdown Previewer
------------------
https://visualstudiogallery.msdn.microsoft.com/0855e23e-4c4c-4c82-8b39-24ab5c5a7f79

The chrome markdown preview was soooo flaky, gave up on it, although it would have been convenient.

And there were 85 markdown previewers to choose from. As alinux user, I'm getting used to an abundance of broken/useless/limited software crap.


CRCl Program 
------------

    CrclDelegateInterface::SetLengthUnits
    CrclDelegateInterface::SetTransSpeed
    CrclDelegateInterface::SetTransAccel
    CrclDelegateInterface::SetEndPoseTolerance
    CrclDelegateInterface::SetIntermediatePoseTolerance
    CrclDelegateInterface::StopMotion
    CrclDelegateInterface::MoveThroughTo
    GotoCRCL Pose 1.5,1,1,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 1500:1000:1000
    Rotation = 180:-0:0
    GotoCRCL Pose 1.5,1,0.0001,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 1500:1000:0.1
    Rotation = 180:-0:0
    CrclDelegateInterface::StopMotion
    CrclDelegateInterface::MoveThroughTo
    GotoCRCL Pose 1.5,1,1,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 1500:1000:1000
    Rotation = 180:-0:0
    GotoCRCL Pose 4,1,1,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 4000:1000:1000
    Rotation = 180:-0:0
    GotoCRCL Pose 4,1,0.5001,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 4000:1000:500.1
    Rotation = 180:-0:0
    CrclDelegateInterface::StopMotion
    CrclDelegateInterface::MoveThroughTo
    GotoCRCL Pose 4,1,1,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 4000:1000:1000
    Rotation = 180:-0:0
    GotoCRCL Pose 8.25,1,1,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 8250:1000:1000
    Rotation = 180:-0:0
    GotoCRCL Pose 8.25,1,0.4,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 8250:1000:400
    Rotation = 180:-0:0
    CrclDelegateInterface::OpenToolChanger
    CrclDelegateInterface::MoveThroughTo
    GotoCRCL Pose 8.25,1,1,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 8250:1000:1000
    Rotation = 180:-0:0
    GotoCRCL Pose 8.75,1,1,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 8750:1000:1000
    Rotation = 180:-0:0
    GotoCRCL Pose 8.75,1,0.5,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 8750:1000:500
    Rotation = 180:-0:0
    CrclDelegateInterface::CloseToolChanger
    CrclDelegateInterface::MoveThroughTo
    GotoCRCL Pose 8.75,1,1,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 8750:1000:1000
    Rotation = 180:-0:0
    GotoCRCL Pose 5.659,1.1,1.8,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 5659:1100:1800
    Rotation = 180:-0:0
    GotoCRCL Pose 5.659,1.1,0.1501,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 5659:1100:150.1
    Rotation = 180:-0:0
    CrclDelegateInterface::StopMotion
    CrclDelegateInterface::MoveThroughTo
    GotoCRCL Pose 5.659,1.1,0.5,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 5659:1100:500
    Rotation = 180:-0:0
    GotoCRCL Pose 3.86,1.07,1,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 3860:1070:1000
    Rotation = 180:-0:0
    GotoCRCL Pose 3.86,1.07,0.6501,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 3860:1070:650.1
    Rotation = 180:-0:0
    CrclDelegateInterface::StopMotion
    CrclDelegateInterface::MoveThroughTo
    GotoCRCL Pose 3.86,1.07,1,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 3860:1070:1000
    Rotation = 180:-0:0
    GotoCRCL Pose 5.659,0.9,0.5,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 5659:900:500
    Rotation = 180:-0:0
    GotoCRCL Pose 5.659,0.9,0.1501,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 5659:900:150.1
    Rotation = 180:-0:0
    CrclDelegateInterface::StopMotion
    CrclDelegateInterface::MoveThroughTo
    GotoCRCL Pose 5.659,0.9,0.5,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 5659:900:500
    Rotation = 180:-0:0
    GotoCRCL Pose 3.86,0.93,1,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 3860:930:1000
    Rotation = 180:-0:0
    GotoCRCL Pose 3.86,0.93,0.6501,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 3860:930:650.1
    Rotation = 180:-0:0
    CrclDelegateInterface::StopMotion
    CrclDelegateInterface::MoveThroughTo
    GotoCRCL Pose 3.86,0.93,1,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 3860:930:1000
    Rotation = 180:-0:0
    GotoCRCL Pose 6.42,1,0.5,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 6420:1000:500
    Rotation = 180:-0:0
    GotoCRCL Pose 6.42,1,0.1501,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 6420:1000:150.1
    Rotation = 180:-0:0
    CrclDelegateInterface::StopMotion
    CrclDelegateInterface::MoveThroughTo
    GotoCRCL Pose 6.42,1,0.5,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 6420:1000:500
    Rotation = 180:-0:0
    GotoCRCL Pose 4.14,0.93,1,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 4140:930:1000
    Rotation = 180:-0:0
    GotoCRCL Pose 4.14,0.93,0.6501,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 4140:930:650.1
    Rotation = 180:-0:0
    CrclDelegateInterface::StopMotion
    CrclDelegateInterface::MoveThroughTo
    GotoCRCL Pose 4.14,0.93,1,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 4140:930:1000
    Rotation = 180:-0:0
    GotoCRCL Pose 7.61,1.02,0.5,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 7610:1020:500
    Rotation = 180:-0:0
    GotoCRCL Pose 7.61,1.02,0.1501,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 7610:1020:150.1
    Rotation = 180:-0:0
    CrclDelegateInterface::StopMotion
    CrclDelegateInterface::MoveThroughTo
    GotoCRCL Pose 7.61,1.02,0.5,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 7610:1020:500
    Rotation = 180:-0:0
    GotoCRCL Pose 4.14,1.07,1,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 4140:1070:1000
    Rotation = 180:-0:0
    GotoCRCL Pose 4.14,1.07,0.6501,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 4140:1070:650.1
    Rotation = 180:-0:0
    CrclDelegateInterface::StopMotion
    CrclDelegateInterface::MoveThroughTo
    GotoCRCL Pose 4.14,1.07,1,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 4140:1070:1000
    Rotation = 180:-0:0
    GotoCRCL Pose 8.75,1,1,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 8750:1000:1000
    Rotation = 180:-0:0
    GotoCRCL Pose 8.75,1,0.475,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 8750:1000:475
    Rotation = 180:-0:0
    CrclDelegateInterface::OpenToolChanger
    CrclDelegateInterface::MoveThroughTo
    GotoCRCL Pose 8.75,1,1,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 8750:1000:1000
    Rotation = 180:-0:0
    GotoCRCL Pose 8.25,1,1,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 8250:1000:1000
    Rotation = 180:-0:0
    GotoCRCL Pose 8.25,1,0.5,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 8250:1000:500
    Rotation = 180:-0:0
    CrclDelegateInterface::CloseToolChanger
    CrclDelegateInterface::MoveThroughTo
    GotoCRCL Pose 8.25,1,1,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 8250:1000:1000
    Rotation = 180:-0:0
    GotoCRCL Pose 4,1,1,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 4000:1000:1000
    Rotation = 180:-0:0
    GotoCRCL Pose 4,1,0.5001,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 4000:1000:500.1
    Rotation = 180:-0:0
    CrclDelegateInterface::StopMotion
    CrclDelegateInterface::MoveThroughTo
    GotoCRCL Pose 4,1,1,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 4000:1000:1000
    Rotation = 180:-0:0
    GotoCRCL Pose 2.5,1,1,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 2500:1000:1000
    Rotation = 180:-0:0
    GotoCRCL Pose 2.5,1,0.0001,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 2500:1000:0.1
    Rotation = 180:-0:0
    CrclDelegateInterface::StopMotion
    CrclDelegateInterface::MoveThroughTo
    GotoCRCL Pose 2.5,1,1,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 2500:1000:1000
    Rotation = 180:-0:0
    GotoCRCL Pose 0.5,0,2,1,0,0,0,0,-1,
    Goto urdf Pose Translation = 500:0:2000
    Rotation = 180:-0:0


Where does the fanuc robot go?
---------------------------

    (0,0,0,0,0k,0)
    .465 0 .695 0 0 1 1 0 0  (180 -90 0)

    (33,0,0,0,0,0)
    .39 .253 .695  0 .545 .839 1  0 0   (90 -57 90)


    CrclDelegateInterface::MoveTo
    GotoCRCL Pose 0.39,0.253,0.695,0,0.545,0.839,1,0,0,
    Goto urdf Pose Translation = 390:253:695
    Rotation = 180:-89.9853:-180

==========================================

    CrclDelegateInterface::SetLengthUnits
    CrclDelegateInterface::SetTransSpeed
    CrclDelegateInterface::SetTransAccel
    CrclDelegateInterface::SetEndPoseTolerance
    CrclDelegateInterface::SetIntermediatePoseTolerance
    CrclDelegateInterface::SetEndEffector
    CrclDelegateInterface::MoveTo
    GotoCRCL Pose 0.39,0.253,0.695,0,0.545,0.839,1,0,0,
    Goto urdf Pose Translation = 390:253:695
    Rotation = 180:-89.9853:-180
    CrclDelegateInterface::Dwell

    RCSInterpreter::ParseCommand
    RCSInterpreter::ParseCommand
    RCSInterpreter::ParseCommand
    Current Pose Translation = 464.975:-0.00919851:695.009
    Rotation = -150.214:-89.9977:-29.7826
    Goal Pose Translation = 390:253:695
    Rotation = 180:-89.9853:-180
    CartesianMotion  Poses Translation = 464.975:-0.00919851:695.009
    Rotation = -150.214:-89.9977:-29.7826
    CartesianMotion  Poses Translation = 439.983:84.3272:695.006
    Rotation = -163.179:-89.9993:-113.995
    CartesianMotion  Poses Translation = 414.992:168.664:695.003
    Rotation = -176.021:-89.9941:-177.603
    CartesianMotion  Poses Translation = 390:253:695
    Rotation = 180:-89.9853:-180
    New IK Joints -1.97821e-05:-7.22635e-05:-3.84365e-05:6.22758e-05:-6.83954e-05:-1.77077e-06:
    GotoPose Translation = 464.975:-0.00919851:695.009
    Rotation = -150.214:-89.9977:-29.7826
    New Joints -1.97821e-05:-7.22635e-05:-3.84365e-05:6.22758e-05:-6.83954e-05:-1.77077e-06:
    New IK Joints -2.91129:-1.44586:0.298472:0.234648:1.40243:-3.29449:
    GotoPose Translation = 439.983:84.3272:695.006
    Rotation = -163.179:-89.9993:-113.995
    New Joints -2.91129:-1.44586:0.298472:0.234648:1.40243:-3.29449:
    New IK Joints -2.67506:-1.44189:0.319451:0.474739:1.39747:-3.43398:
    GotoPose Translation = 414.992:168.664:695.003
    Rotation = -176.021:-89.9941:-177.603
    New Joints -2.67506:-1.44189:0.319451:0.474739:1.39747:-3.43398:
    New IK Joints -2.45751:-1.41568:0.427933:0.701811:1.34968:-3.54969:
    GotoPose Translation = 390:253:695
    Rotation = 180:-89.9853:-180
    New Joints -2.45751:-1.41568:0.427933:0.701811:1.34968:-3.54969:
    New Joint Position -1.97821e-05:-7.22635e-05:-3.84365e-05:6.22758e-05:-6.83954e-05:-1.77077e-06:
    New Joint Position -2.91129:-1.44586:0.298472:0.234648:1.40243:-3.29449:
    New Joint Position -2.67506:-1.44189:0.319451:0.474739:1.39747:-3.43398:
    New Joint Position -2.45751:-1.41568:0.427933:0.701811:1.34968:-3.54969:
    RCSInterpreter::ParseCommand


    Current joints=0:0:0:0:0:0:
    CrclDelegateInterface::SetAngleUnitsDEGREE
    Current=0:0:0:0:0:0:
    Translation =    465.0000:     0.0000:   695.0000
    CrclDelegateInterface::SetLengthUnits=meter
    CrclDelegateInterface::SetTransSpeed
    CrclDelegateInterface::SetTransAccel
    CrclDelegateInterface::SetEndPoseTolerance
    CrclDelegateInterface::SetIntermediatePoseTolerance
    CrclDelegateInterface::SetEndEffector= 0.00
    CrclDelegateInterface::MoveTo
    GotoCRCL Pose 0.39,0.253,0.695,0,0.545,0.839,1,0,0,
    Goto urdf Pose Translation = 390:253:695
    Rotation = 180:-89.9853:-180

THIS TIME IK WORKED????!!!!??!!


    CrclDelegateInterface::Dwell=%5.2
    RCSInterpreter::ParseCommand
    RCSInterpreter::ParseCommand
    RCSInterpreter::ParseCommand
    Current Pose Translation = 465:0:695
    Rotation = 180:-90:0
    Goal Pose Translation = 390:253:695
    Rotation = 180:-89.9853:-180
    CartesianMotion  Poses Translation = 465:0:695
    Rotation = 180:-90:0
    CartesianMotion  Poses Translation = 440:84.3333:695
    Rotation = -180:-89.9983:-180
    CartesianMotion  Poses Translation = 415:168.667:695
    Rotation = -180:-89.9933:-180
    CartesianMotion  Poses Translation = 390:253:695
    Rotation = 180:-89.9853:-180
    New IK Joints 0:0:0:0:0:0:
    GotoPose Translation = 465:0:695
    Rotation = 180:-90:0
    New Joints 0:0:0:0:0:0:
    New IK Joints 0.22962:-0.0461163:-0.0437904:-1.60459:0.227424:-1.79647:
    GotoPose Translation = 440:84.3333:695
    Rotation = -180:-89.9983:-180
    New Joints 0.22962:-0.0461163:-0.0437904:-1.60459:0.227424:-1.79647:
    New IK Joints 0.466449:-0.0303704:-0.0331881:-1.53475:0.466877:-1.91761:
    GotoPose Translation = 415:168.667:695
    Rotation = -180:-89.9933:-180
    New Joints 0.466449:-0.0303704:-0.0331881:-1.53475:0.466877:-1.91761:
    New IK Joints 0.684214:0.0459945:0.0494784:-1.59129:0.68255:-2.1703:
    GotoPose Translation = 390:253:695
    Rotation = 180:-89.9853:-180
    New Joints 0.684214:0.0459945:0.0494784:-1.59129:0.68255:-2.1703:
    New Joint Position 0:0:0:0:0:0:
    New Joint Position 0.22962:-0.0461163:-0.0437904:-1.60459:0.227424:-1.79647:
    New Joint Position 0.466449:-0.0303704:-0.0331881:-1.53475:0.466877:-1.91761:
    New Joint Position 0.684214:0.0459945:0.0494784:-1.59129:0.68255:-2.1703:
    RCSInterpreter::ParseCommand





Setting Hard/Soft Joint Limits Dynamically
----------------------------------------------
https://github.com/ros-controls/ros_control/wiki/joint_limits_interface

But of course, if you use moveit this is completely null and void! Beautiful modular code.

    #include <moveit/robot_model_loader/robot_model_loader.h>
    #include <moveit/robot_model/robot_model.h>

    void GetJointLimits(std::vector<std::string> names, std::vector<double> lower, std::vector<double> upper) {
         boost::shared_ptr<robot_model_loader::RobotModelLoader>  urdf(new robot_model_loader::RobotModelLoader("robot_description"));
         
         
         moveit::core::JointModel* jm = urdf->getModel()->getJointModel("joint_1");
    ...
    }

Mr Dave Coleman
--------------
https://github.com/davetcoleman?tab=repositories

Install rqt-moveit
-----------------
http://wiki.ros.org/rqt/UserGuide/Install/Groovy

rostopic echo rviz_rufous_20055_8662231662545915181/motionplanning_planning_scene_monitor/parameter_descriptions
------------------------------------------------------------------------------------------------------------------

    michalos@rufous:~/catkin_ws$ rostopic echo rviz_rufous_20055_8662231662545915181/motionplanning_planning_scene_monitor/parameter_descriptions
    groups: 
      - 
        name: Default
        type: ''
        parameters: 
          - 
            name: publish_planning_scene
            type: bool
            level: 1
            description: Set to True to publish Planning Scenes
            edit_method: ''
          - 
            name: publish_planning_scene_hz
            type: double
            level: 2
            description: Set the maximum frequency at which planning scene updates are published
            edit_method: ''
          - 
            name: publish_geometry_updates
            type: bool
            level: 3
            description: Set to True to publish geometry updates of the planning scene
            edit_method: ''
          - 
            name: publish_state_updates
            type: bool
            level: 4
            description: Set to True to publish geometry updates of the planning scene
            edit_method: ''
          - 
            name: publish_transforms_updates
            type: bool
            level: 5
            description: Set to True to publish geometry updates of the planning scene
            edit_method: ''
        parent: 0
        id: 0
    max: 
      bools: 
        - 
          name: publish_planning_scene
          value: True
        - 
          name: publish_geometry_updates
          value: True
        - 
          name: publish_state_updates
          value: True
        - 
          name: publish_transforms_updates
          value: True
      ints: []
      strs: []
      doubles: 
        - 
          name: publish_planning_scene_hz
          value: 100.0
      groups: 
        - 
          name: Default
          state: True
          id: 0
          parent: 0
    min: 
      bools: 
        - 
          name: publish_planning_scene
          value: False
        - 
          name: publish_geometry_updates
          value: False
        - 
          name: publish_state_updates
          value: False
        - 
          name: publish_transforms_updates
          value: False
      ints: []
      strs: []
      doubles: 
        - 
          name: publish_planning_scene_hz
          value: 0.1
      groups: 
        - 
          name: Default
          state: True
          id: 0
          parent: 0
    dflt: 
      bools: 
        - 
          name: publish_planning_scene
          value: False
        - 
          name: publish_geometry_updates
          value: True
        - 
          name: publish_state_updates
          value: False
        - 
          name: publish_transforms_updates
          value: False
      ints: []
      strs: []
      doubles: 
        - 
          name: publish_planning_scene_hz
          value: 4.0
      groups: 
        - 
          name: Default
          state: True
          id: 0
          parent: 0
    ---



 catkin_make clean
-----------------
problem with controller.o linking


rufous disk problem
----------------------
Clean the disk
    fsck -As

Had to force a mount
    mount -o remount,rw /
    then deleted trash:
    cd ~/michalos/.local/share/Trash
    rm -rf files/*.*

    sudo apt-get autoremove (problem with kuka ros downloads)

Then removed excess 12.04 kernal images

    cd /boot
    sudo rm *.3.13.0-58*


github
-----------
    git commit -a (added files)
normally git commit .
VIM nightmare: esc and :wq  (write and quit)


    michalos@rufous:~/github/usnistgov/el-robotics-core$ git push origin master
    Username for 'https://github.com': johnmichaloski
    Password for 'https://johnmichaloski@github.com': 
    Counting objects: 10, done.
    Delta compression using up to 8 threads.
    Compressing objects: 100% (2/2), done.
    Writing objects: 100% (2/2), 264 bytes | 0 bytes/s, done.
    Total 2 (delta 1), reused 0 (delta 0)
    To https://github.com/usnistgov/el-robotics-core
       395d561..b74a274  master -> master
    michalos@rufous:~/github/usnistgov/el-robotics-core$ 

Did not add nist_fanuc:

    git add nist_fanuc
    git commit .
    git push origin master

Adding doxygen files and committing with message without using vim:

    git add .
    git status
    git commit -m "add doxygen files"
    git push origin master



git remote changes and incorporate into local repository
-------------------------------------------------------------
You cannot just "git fetch" remote changes, they will not be incorporated into your
local repository unless you **merge** them. So must use "git merge remotebranchname"

    https://help.github.com/articles/fetching-a-remote/

    michalos@rufous:~/github/usnistgov/el-robotics-core$ git merge origin
    Updating 5f50122..bfc7439
    Fast-forward
     nist_fanuc/CMakeLists.txt           |   7 ++-
     nist_fanuc/scripts/runrvizdemo.bash |   2 +-
     nist_kitting/src/move_group.cpp     |  59 +------------------
     nist_kitting/src/mover.cpp          | 111 ++++--------------------------------
     ulapi/package.xml                   |   2 +-
     5 files changed, 20 insertions(+), 161 deletions(-)
    michalos@rufous:~/github/usnistgov/el-robotics-core$ 





creating doxygen documentation
----------------------------------------
1) Install
sudo apt-get install ros-indigo-rosdoc-lite 
2) Run rosdoc-lite
cd src/nist_fanuc
rosdoc_lite ../nist_fanuc
3) Output in doc
4) cd doc/html  then double click index.html

Problems with exclude, so hard coded doxygen ....


Catching the Signal ^C
----------------------------
This signal interrupt handling worked as long as there were no background threads...

    void signal_callback_handler(int signum) {
        /* NOTE some versions of UNIX will reset signal to default
        after each call. So for portability reset signal each time */
        RCS::Controller.bMainLoop = false;
        std::cout << "you have pressed ctrl-c \n";
        signal(SIGINT, signal_callback_handler); /*  */

    }
    main() 
    {
        struct sigaction sigIntHandler;

        sigIntHandler.sa_handler = signal_callback_handler;
        sigemptyset(&sigIntHandler.sa_mask);
        sigIntHandler.sa_flags = 0;

        sigaction(SIGINT, &sigIntHandler, NULL);

        while(RCS::Controller.bMainLoop)
        {
            Globals.Sleep(1000);
        }
        return 0;


Yikes! From  http://wiki.ros.org/roscpp/Overview/Initialization%20and%20Shutdown

By default roscpp also installs a SIGINT handler which will detect Ctrl-C and automatically shutdown for you.
J
roscpp/Overview/Initialization and Shutdown - ROS Wiki
wiki.ros.org
Initialization. There are two levels of initialization for a roscpp Node: Initializing the node through a call to one of the ros::init() functions.

By default roscpp also installs a SIGINT handler which will detect Ctrl-C and automatically shutdown for you.


Testing for Shutdown

There are two methods to check for various states of shutdown. The most common is ros::ok(). Once ros::ok()returns false, the node has finished shutting down. A common use of ros::ok():
Toggle line numbers

       1 while (ros::ok())
       2 {
       3   ...
       4 }

If you want to write your own sigint handler instead of using ROS:

    static bool bMainLoop=true;
    void mySigintHandler(int sig)
    {
      // Do some custom action.
      // For example, publish a stop message to some other nodes.
       std::cout << "Received Cntl c" << std::flush;
      // All the default sigint handler does is call shutdown()
      bMainLoop=false;
     
    }
    main() { ...
    ros::init(argc, argv, "nist_fanuc", ros::init_options::NoSigintHandler);
    signal(SIGINT, mySigintHandler);





test crcl math code
----------------------
1) Add /src folder,
2) Create testcrclmath.cpp file:

    #include <gtest/gtest.h>

    TEST(TestSuite, testCase1) {
        ASSERT_EQ(1, 1) << "Vectors 1 and 1 not equal";
    }

    int main(int argc, char ** argv) {

        testing::InitGoogleTest(&argc, argv);
        return RUN_ALL_TESTS();
    }
3) Modify  CMakeLists.txt (catkin will handle rest of stuff)
catkin_add_gtest(crclmathunittest test/crclmathtest.cpp)

4) Run individual gtest in package
catkin_make run_tests_nist_fanuc


Turn off auto paren completion in Netbeans - really annoying
---------------------------------------
Tools->Options

Editor->Code Completion


tf include locatino
--------------------
/opt/ros/indigo/include/tf/LinearMath

Backup all of home directory
-----------------------------
cp -r /home/michalos/*  "/media/michalos/FreeAgent GoFlex Drive/WorkBackup"


Install python pip - risky
--------------------------------
sudo apt-get install python-pip

michalos@rufous:~/catkin_ws/src/nist_fanuc/nodes$ pip install mathutils
Downloading/unpacking mathutils
  Downloading mathutils-2.76.tar.gz (190kB): 190kB downloaded
  Running setup.py (path:/tmp/pip_build_michalos/mathutils/setup.py) egg_info for package mathutils
    Sorry, Python 2 are not supported
Cleaning up...
No files/directories in /tmp/pip_build_michalos/mathutils/pip-egg-info (from PKG-INFO)
Storing debug log for failure in /home/michalos/.pip/pip.log
michalos@rufous:~/catkin_ws/src/nist_fanuc/nodes$ 


python socket 
---------------
tcp      591      0 localhost:64444         localhost:46638         CLOSE_WAIT 
tcp        1      0 localhost:64444         localhost:46670         CLOSE_WAIT 
tcp        0      0 localhost:64444         localhost:46842         ESTABLISHED
tcp        0      0 localhost:46842         localhost:64444         ESTABLISHED
tcp        1      0 localhost:64444         localhost:46637         CLOSE_WAIT 
tcp      627      0 localhost:64444         localhost:46671         CLOSE_WAIT 
tcp     3481      0 localhost:64444         localhost:46633         CLOSE_WAIT 
tcp      628      0 localhost:64444         localhost:46664         CLOSE_WAIT

After ^C python socket program
michalos@rufous:~/catkin_ws/src/nist_fanuc/nodes$ netstat| grep 64444
tcp      591      0 localhost:64444         localhost:46638         CLOSE_WAIT 
tcp        1      0 localhost:64444         localhost:46670         CLOSE_WAIT 
tcp        1      0 localhost:64444         localhost:46842         CLOSE_WAIT 
tcp        0      0 localhost:46842         localhost:64444         FIN_WAIT2  
tcp        1      0 localhost:64444         localhost:46637         CLOSE_WAIT 
tcp      627      0 localhost:64444         localhost:46671         CLOSE_WAIT 
tcp     3481      0 localhost:64444         localhost:46633         CLOSE_WAIT 
tcp      628      0 localhost:64444         localhost:46664         CLOSE_WAIT 




Netbeans DebuggingExceptionBreakpoint
-----------------------------------------
Exception Breakpoints

It's often useful to stop execution of an application when an exception occurs, and inspect the state of the app in debugger. It is very simple to do: setup an exception breakpoint and just run the application via debugger (or attach to an running application).

To setup an exception breakpoint:
Go to menu Debug | New Breakpoint (Ctrl+Shift+F8).
In the New Breakpoint dialog select the Exception breakpoint type from the combobox.
Enter the exception to track (fully qualified class name).
You can modify the other properties too.
Typically you want to watch for some general exception superclass, e.g. java.lang.Exception, or exceptions that do not have to be handled, e.g. java.lang.AssertionError, or java.lang.RuntimeException.
In a bigger application you may want to narrow the scope of classes to watch for exceptions only from your classes, not to stop on exceptions from JDK internals or other code. You can set a match or exclude class filter in the Filter on Classes Throwing the Exception option of the breakpoint dialog.

To turn off breakpoint:
Select menu Window / Debugging / Breakpoints (or press Alt + Shift + 5), then right-click in the Breakpoints window and select Delete All.



Movegroup died ...
---------------
[move_group-5] process has died [pid 4186, exit code -11, cmd /opt/ros/indigo/lib/moveit_ros_move_group/move_group __name:=move_group __log:=/home/michalos/.ros/log/6e9469f8-f05e-11e5-9a07-ecf4bb31ca6d/move_group-5.log].
log file: /home/michalos/.ros/log/6e9469f8-f05e-11e5-9a07-ecf4bb31ca6d/move_group-5*.log


Problems with catkin now
In file included from /opt/ros/indigo/include/moveit/robot_model/joint_model_group.h:41:0,
                 from /opt/ros/indigo/include/moveit/robot_model/robot_model.h:48,
                 from /opt/ros/indigo/include/moveit/robot_model_loader/robot_model_loader.h:40,
                 from /usr/local/michalos/nistfanuc_ws/src/nist_fanuc/include/nist_fanuc/RCS.h:39,
                 from /usr/local/michalos/nistfanuc_ws/src/nist_fanuc/include/nist_fanuc/crcl.h:18,
                 from /usr/local/michalos/nistfanuc_ws/src/nist_fanuc/include/nist_fanuc/CrclInterface.h:16,
                 from /usr/local/michalos/nistfanuc_ws/src/nist_fanuc/include/nist_fanuc/AsioCrclServer.h:29,
                 from /usr/local/michalos/nistfanuc_ws/src/nist_fanuc/include/nist_fanuc/Controller.h:18,
                 from /usr/local/michalos/nistfanuc_ws/src/nist_fanuc/src/Controller.cpp:22:
/opt/ros/indigo/include/moveit/robot_model/joint_model.h:47:26: fatal error: Eigen/Geometry: No such file or directory
Sigh.


<depend package="tf"/>

I installed libeigen3-dev. Now I used

sudo find /usr -name eigen*

and found that eigen is installed in /usr/include/eigen3/Eigen. So apparently each installation goes different.

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    # include breakdown for this project
    include/${PROJECT_NAME}
    include/${PROJECT_NAME}/CrclXsd
    include/${PROJECT_NAME}/NIST

   ${descartes_moveit_INCLUDE_DIRS}
   ${descartes_trajectory_INCLUDE_DIRS}
   ${descartes_planner_INCLUDE_DIRS}
   ${descartes_core_INCLUDE_DIRS}
   /usr/include/eigen3
)

Started using Python catkin tools
------------------------------------

Installed python catkin tools - supposedly handle single package project easier in Qt Creator

	sudo apt-get install python-catkin-tools

You must manually cd to the location, don't use any symlinks.

	cd /usr/local/michalos

Then re inited catkin package with new tools:
	catkin clean --all
	catkin init
	qtcreator

With the new python catkin_tools, there is no longer a top level make file for the whole workspace. Instead, open each package as an individual project in QtCreator. The trick is to set the build folder to ws/build/your_package instead of ws/build as before. 



Disable qt creator auto build when debug
Navigate to Tools/Options. Somewhre Uncheck "Always build project before running". Back to unknown error.


Using Python catkin tools and debugging package in QT Creator
----------------------------------------------------------------

Hey, there is another post in ROS Answers addressing how to debug a rosnode in Qtcreator: http://answers.ros.org/question/34966...

I think that the answer can be summarized to:

    compile the node/package that you want to compile in debug mode (add "set(CMAKE_BUILD_TYPE Debug)" to your CMakeLists.txt)
    start your roscore and everything else as usual, except for the node you want to debug
    start qtcreator with "sudo" (remember to source correctly your catkin workspace)
    in qtcreator start the debugging mode for the rosnode by going to "Debug"->"Start Debugging"->"Attach to unstarted Application...", look for your compiled node, it should be in: "${CATKIN_WORKSPACE_FOLDER}"/devel/lib/package_name/node_name and click "Start Watching".
    start the node in a terminal either with rosrun or roslaunch
    enjoy...

Note: you need to run qtcreator with "sudo", otherwise you get a "ptrace operation not allowed" problem. Note2: if you try to run qtcreator as a normal user afterwards, you will have problems accessing some of the Qt configuration files of your home folder, run these commands to get this back to normal:

sudo chown -R ${USER}:${USER} .qt
sudo chown -R ${USER}:${USER} .config/

Now can't see headers in Qt Project
---------------------------------------
http://answers.ros.org/question/56685/is-there-any-way-to-get-qt-creator-to-show-all-of-a-projects-subdirectories/

've got many header files *.h and *.hxx which are included in several *.cpp files but do not show up in the project tree on qtcreator, is there any way of telling qtcreator that those files do belong to the project?
set( MY_SRCS
include/version.h
include/idcache.h
src/json.cpp
include/json.h
src/diskinfo.cpp
include/diskinfo.h
src/exif.cpp
include/exif.h
include/networkmanager.h
include/networkmanager_p.h
)

#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB children ${CMAKE_SOURCE_DIR}/*)
FOREACH(child ${children})
  IF(IS_DIRECTORY ${child})
    file(GLOB_RECURSE dir_files "${child}/*")
    LIST(APPEND extra_files ${dir_files})
  ENDIF()
ENDFOREACH()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})




Beautify Code in QT Creator
-------------------------------
^A ^I - only  indents - useless

http://doc.qt.io/qtcreator/creator-beautifier.html

Load config file into rviz with roslaunch
---------------------------------------------
Use the command line parameter -d <arg> at startup. <arg> is the path to your .vcg-file:

	rosrun rviz rviz -d <arg>

For day-to-day use, rviz stores your current config in $HOME/.rviz/display_config, so you can close and re-open rviz without losing your displays and layout.


Problems with Markdown using email from WIndows to Unix
---------------------------
sudo apt-get install dos2unix

Problem was double line feeds?

  525  tr -s '\n' < QtCreatorWIthRos.md > Qt.md
  526  hexdump -c Qt.md
  527  ls
  528  mv Qt.md QtCreatorWIthRos.md



FInding contents of tool0 cartesian position as shown in rviz
-------------------------------------------------
Tue Apr 12 15:30:43 EDT 2016 

 507  rostopic type /move_group/goal
  508  rosmsg show moveit_msgs/MoveGroupActionGoal


$rostopic echo /robot_status
--
header: 
  seq: 841725
  stamp: 
    secs: 0
    nsecs: 0
  frame_id: ''
mode: 
  val: 2
e_stopped: 
  val: 0
drives_powered: 
  val: 1
motion_possible: 
  val: 1
in_motion: 
  val: 0
in_error: 
  val: 0
error_code: 0
---

michalos@rufous:nistfanuc_ws> rostopic echo /move_group/goal

header: 
  seq: 0
  stamp: 
    secs: 1460490263
    nsecs: 207782574
  frame_id: ''
goal_id: 
  stamp: 
    secs: 1460490263
    nsecs: 207787782
  id: /rviz_rufous_6309_8934923881601796331-1-1460490263.207787782
goal: 
  request: 
    workspace_parameters: 
      header: 
        seq: 0
        stamp: 
          secs: 1460490263
          nsecs: 207291483
        frame_id: /base_link
      min_corner: 
        x: -1.0
        y: -1.0
        z: -1.0
      max_corner: 
        x: 1.0
        y: 1.0
        z: 1.0
    start_state: 
      joint_state: 
        header: 
          seq: 0
          stamp: 
            secs: 0
            nsecs: 0
          frame_id: /base_link
        name: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        velocity: []
        effort: []
      multi_dof_joint_state: 
        header: 
          seq: 0
          stamp: 
            secs: 0
            nsecs: 0
          frame_id: /base_link
        joint_names: []
        transforms: []
        twist: []
        wrench: []
      attached_collision_objects: []
      is_diff: False
    goal_constraints: 
      - 
        name: ''
        joint_constraints: 
          - 
            joint_name: joint_1
            position: 2.47973143673
            tolerance_above: 0.0001
            tolerance_below: 0.0001
            weight: 1.0
          - 
            joint_name: joint_2
            position: -1.20101259358
            tolerance_above: 0.0001
            tolerance_below: 0.0001
            weight: 1.0
          - 
            joint_name: joint_3
            position: 1.01879929359
            tolerance_above: 0.0001
            tolerance_below: 0.0001
            weight: 1.0
          - 
            joint_name: joint_4
            position: -0.774237325468
            tolerance_above: 0.0001
            tolerance_below: 0.0001
            weight: 1.0
          - 
            joint_name: joint_5
            position: 1.0738053828
            tolerance_above: 0.0001
            tolerance_below: 0.0001
            weight: 1.0
          - 
            joint_name: joint_6
            position: 2.70530184851
            tolerance_above: 0.0001
            tolerance_below: 0.0001
            weight: 1.0
        position_constraints: []
        orientation_constraints: []
        visibility_constraints: []
    path_constraints: 
      name: ''
      joint_constraints: []
      position_constraints: []
      orientation_constraints: []
      visibility_constraints: []
    trajectory_constraints: 
      constraints: []
    planner_id: ''
    group_name: manipulator
    num_planning_attempts: 10
    allowed_planning_time: 5.0
    max_velocity_scaling_factor: 1.0
  planning_options: 
    planning_scene_diff: 
      name: ''
      robot_state: 
        joint_state: 
          header: 
            seq: 0
            stamp: 
              secs: 0
              nsecs: 0
            frame_id: ''
          name: []
          position: []
          velocity: []
          effort: []
        multi_dof_joint_state: 
          header: 
            seq: 0
            stamp: 
              secs: 0
              nsecs: 0
            frame_id: ''
          joint_names: []
          transforms: []
          twist: []
          wrench: []
        attached_collision_objects: []
        is_diff: True
      robot_model_name: ''
      fixed_frame_transforms: []
      allowed_collision_matrix: 
        entry_names: []
        entry_values: []
        default_entry_names: []
        default_entry_values: []
      link_padding: []
      link_scale: []
      object_colors: []
      world: 
        collision_objects: []
        octomap: 
          header: 
            seq: 0
            stamp: 
              secs: 0
              nsecs: 0
            frame_id: ''
          origin: 
            position: 
              x: 0.0
              y: 0.0
              z: 0.0
            orientation: 
              x: 0.0
              y: 0.0
              z: 0.0
              w: 0.0
          octomap: 
            header: 
              seq: 0
              stamp: 
                secs: 0
                nsecs: 0
              frame_id: ''
            binary: False
            id: ''
            resolution: 0.0
            data: []
      is_diff: True
    plan_only: False
    look_around: False
    look_around_attempts: 0
    max_safe_execution_cost: 0.0
    replan: False
    replan_attempts: 0
    replan_delay: 2.0
---

Using python and tf to get latest tool0 link trans/rot
--------------------------------------------------------

tf.ExtrapolationException: Lookup would require extrapolation into the future.  Requested time 1460650813.829602003 but the latest data is at time 1460650813.826994181, when looking up transform from frame [tool0] to frame [base_link]




github creation of 1.1 branch to match crcl verion
--------------------------------------------------
Fri Apr 15 14:19:06 EDT 2016 

	590  git checkout -b 1.1
	591  git push origin 1.1

	640  git add -A .
	641  git commit -m "added ee tolerance - not used"
	642  git push origin 1.1



