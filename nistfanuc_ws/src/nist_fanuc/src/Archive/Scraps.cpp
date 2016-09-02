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


#if 0
#include <rviz_visual_tools/rviz_visual_tools.h>
#include "Globals.h"
#include <boost/bind.hpp>
#include <algorithm>
struct ObjectDB;

#define XYWALL 1
#define XZWALL 2
#define YZWALL 3

bool publishWall(int WallType, const Eigen::Affine3d & inpose, const rviz_visual_tools::colors &color, double scale);
void DrawObject(ObjectDB *obj);
void InitSceneObject();
void UpdateScene(std::string objname, Eigen::Affine3d pose, rviz_visual_tools::colors color );
void SetupSceneObject() ;

struct ObjectDB
{
    Eigen::Affine3d pose;
    std::string name;
    std::string type;
    std::string filepath;
    rviz_visual_tools::colors color;
    double scale;
    std::size_t id;
    int walltype;
     ObjectDB(
            std::string name,
            std::string type,
            std::size_t id=0,
            Eigen::Affine3d pose=Eigen::Affine3d::Identity(),
            std::string filepath="",
            rviz_visual_tools::colors color=rviz_visual_tools::CLEAR,
            double scale = 1.0) {
        this->name = name;
        this->pose = pose;
        this->type = type;
        this->filepath = filepath;
        this->color = color;
        this->scale = scale;
        if(id==0)
            this->id = gid++;
        else this->id = id;
     }

     static void Save( ObjectDB * obj) { objects.push_back(obj); }

     static ObjectDB * Find( std::size_t id) {
        std::vector<ObjectDB *>::iterator it = std::find_if(objects.begin(), objects.end(),
                boost::bind(&ObjectDB::id, _1) == id);

        if (it != objects.end())
            return *it;

        return NULL;
     }

     static Eigen::Affine3d& FindPose(std::string name) {
        std::vector<ObjectDB *>::iterator it = std::find_if(objects.begin(), objects.end(),
                boost::bind(&ObjectDB::name, _1) == name);
        if (it != objects.end())
            return (*it)->pose;
        return dummy->pose;
    }
     static ObjectDB * Find( std::string name) {
        std::vector<ObjectDB *>::iterator it = std::find_if(objects.begin(), objects.end(), 
        boost::bind(&ObjectDB::name, _1) == name); 
        if(it!=objects.end())
            return *it;
        return NULL;
     }

   static std::size_t gid;
   static  std::vector<ObjectDB*> objects;
   static ObjectDB * dummy;
};

#endif
