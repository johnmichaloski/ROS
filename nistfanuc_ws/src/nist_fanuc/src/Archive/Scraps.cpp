//#define MOVEITKIN      
#ifdef MOVEITKIN
        kin = boost::shared_ptr<IKinematics>(new MoveitKinematics(nh));
        // Initializatin of Controller instantiatio of shared objects  
        kin->Init(std::string("manipulator"), std::string("tool0"));
        RCS::Cnc.Kinematics() = kin;
#endif
#ifdef MOVEITPLANNER
        moveitPlanner = boost::shared_ptr<MoveitPlanning>(new MoveitPlanning(nh));
        RCS::Cnc.MoveitPlanner() = moveitPlanner;
#endif

        //       jointWriter->Start();
        std::vector<double> testjoints = ToVector<double>(6, 0.0,0.0,0.0,0.0,0.0,0.0);
        RCS::Pose testpose = kin->FK(testjoints);
        LOG_DEBUG << "Test Pose " << DumpPoseSimple(testpose).c_str();

        //#define ROBOTSTATUS
#ifdef ROBOTSTATUS
        RCS::RobotStatus robotstatus;
        //       robotstatus.CrclDelegate() = crcl;
        robotstatus.JointReader() = jointReader;
        robotstatus.CycleTime() = DEFAULT_LOOP_CYCLE;
#ifdef MOVEITKIN
        robotstatus.Kinematics() = kin;
#endif
        //        robotstatus.Start(); // start the controller status thread
#endif

#if DESCARTES
        boost::shared_ptr<CTrajectory> traj = boost::shared_ptr<CTrajectory> (new CTrajectory());
        // Descarte trajectory writer - uses action lib
        //boost::shared_ptr<CTrajectoryWriter> trajWriter = boost::shared_ptr<CTrajectoryWriter>(new CTrajectoryWriter(traj));

        // INitialize this if you are using Descartes
        // Create a robot model and initialize trajectory with it
        const std::string robot_description = "robot_description";
        const std::string group_name = "manipulator"; // name of the kinematic group
        const std::string world_frame = "/base_link"; // Name of frame in which you are expressing poses.
        const std::string tcp_frame = "tool0"; // tool center point frame
        std::vector<std::string> names;
        // This assumes the yaml file containgin ros param controller_joint_names is loaded
        nh.getParam("controller_joint_names", names);
        RCS::Controller.TrajectoryModel() = traj;
        RCS::Controller.TrajectoryModel()->Init(robot_description, group_name, world_frame, tcp_frame, names);
#endif

