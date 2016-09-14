#if 0
        //http://www.radmangames.com/programming/how-to-use-boost-bind
        RCS::Pose Base(Quaternion(0, 0, 0, 1), Vector3(0, 0, 0));

        RCS::Pose Robot(Quaternion(0, 0, 0, 1), Vector3(0, 0, 0));
        RCS::Pose Gripper(Quaternion(0, 0, 0, 1), Vector3(.120, 0, 0));
        RCS::Pose Table(Quaternion(0, 0, 0, 1), Vector3(0, 0, 0));
        RCS::Pose GoalPose(Quaternion ( Vector3(0, 1, 0), 1.57), Vector3(0.25, -.45, 0.35));
        // Works
        // RCS::Pose GoalPose(Quaternion (0, 0, 0, 1), Vector3(0.465, 0, 0.695));
       // RCS::Pose GoalPose(Quaternion ( Vector3(0, 1, 0), 1.57), Vector3(0.465, 0, 0.695));
        //RCS::Pose GoalPose(Quaternion (0, 0, 0, 1), Vector3(0.465, 0, .335));

        KinematicChain::MotionEquation chain;
        chain.make_equation("Test", //kin,
                KinematicChain::MotionEquation::BASE, 
                KinematicChain::MotionEquation::ROBOT, 
                KinematicChain::MotionEquation::TOOL, 
                KinematicChain::MotionEquation::EQUALS,
                KinematicChain::MotionEquation::TABLE, 
                KinematicChain::MotionEquation::GOAL,
                KinematicChain::MotionEquation::DONE
                );
        //chain.SetPoseCallback(KinematicChain::MotionEquation::GOAL, boost::bind(&KinematicChain::MotionEquation::GetPose, &chain, _1));
        chain.SetPose( KinematicChain::MotionEquation::GOAL, GoalPose);
        //chain.SetPoseCallback(KinematicChain::MotionEquation::GOAL, boost::bind(&KinematicChain::MotionEquation::GetPose, &chain, _1));
        chain.SetPose( KinematicChain::MotionEquation::TOOL, Gripper);
        std::cout << chain.DumpEquation();
       std::vector<double> joints = chain. Solve(kin);
#endif
