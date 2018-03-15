

/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I.
 */
//#pragma message "Compiling " __FILE__ 


#include "nist_robotsnc/Demo.h"
#include "nist_robotsnc/Controller.h"
#include "nist_robotsnc/Globals.h"
#include "nist_robotsnc/Scene.h"
#include "nist_robotsnc/Shape.h"
#include "nist_robotsnc/MotionException.h"

using namespace RCS;
using namespace Conversion;

//#ifndef PI_2
//#define PI_2 1.5707963268
//#endif

boost::mutex RvizDemo::_flag_mutex;


/////////////////////////////////////////////////////////////////////////

GearDemo::GearDemo(ros::NodeHandle & nh, std::string pkgpath, tf::Pose offset) :
_nh(nh), _path(pkgpath), _baseoffset(offset), rvizdemo(nh) {

    _jsonpath = pkgpath + "/config/shapes.json";
    _shapes.ParseJsonFile(_jsonpath);

}

bool GearDemo::IssueRobotCommands(CrclApi & r, bool bSafe) {
    // Finish queuing commands before handling them....
    boost::mutex::scoped_lock lock(cncmutex);
    static double dwelltime = 1.0;
    std::vector<long unsigned int> vjointnum = ToVector<long unsigned int>(6, 0L, 1L, 2L, 3L, 4L, 5L);
    //RCS::Pose retract = RCS::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0.2));

    RCS::CanonCmd cmd;
    RCS::Pose pickpose;

    boost::shared_ptr< ShapeModel::Instance> instance;
    if ((instance = FindFreeGear()) == NULL)
        return false;

    std::string gearname = instance->name;

    // Ok we will move this gear - mark as no longer free standing
    instance->properties["state"] = "stored";

    SceneObject &obj = pScene->Find(instance->name);
    //NC_ASSERT(obj != NULL);

    tf::Pose affpose = obj.pose; // Convert<Eigen::Vector3d, Eigen::Affine3d>(obj.pose.translation()); 
    // The object gripper offset is where on the object it is to be gripped
    tf::Pose gripperoffset = obj.gripperoffset; // Convert<Eigen::Affine3d, tf::Pose>(obj.gripperoffset);
    // THe gripperoffset is the robot gripper offset back to the 0T6 equivalent
    pickpose = RCS::Pose(r.QBend, affpose.getOrigin()) * gripperoffset; // /*Convert<Eigen::Vector3d,tf::Vector3>(affpose.translation()))*gripperoffset;

    tf::Vector3 offset = pickpose.getOrigin();

    // Retract
    r.MoveTo(CrclApi::retract * RCS::Pose(r.QBend, offset));
    r.DoDwell(r.mydwell);
    r.MoveTo(RCS::Pose(r.QBend, offset) * gripperoffset);
    r.DoDwell(r.mydwell);
    r.CloseGripper();
    r.DoDwell(r.mydwell);
    r.MoveTo(CrclApi::retract * RCS::Pose(r.QBend, offset), gearname);

    tf::Pose slotpose;

    try {
        GearDemo::OpenHolderSlot holderslot = FindFreeGearHolder(instance->metatype);
        slotpose = holderslot.second;
        boost::shared_ptr< ShapeModel::Instance> vinstance = holderslot.first;
        tf::Pose gearpose = _baseoffset * _shapes.GetInstancePose(vinstance);
        slotpose = gearpose * slotpose;
    } catch (...) {
        return false;
    }

    RCS::Pose placepose = RCS::Pose(r.QBend, slotpose.getOrigin()); // fixme: what if gear rotated
    r.Place(placepose, gearname);
    r.DoDwell(r.mydwell);

    if (bSafe) {
        std::vector<unsigned long> jointnum(r.cnc()->Kinematics()->NumJoints());
        std::iota(jointnum.begin(), jointnum.end(), 0); // adjusted already to 0..n-1
        r.MoveJoints(jointnum, r.cnc()->NamedJointMove["Safe"]);
    }
    return true;
}

boost::shared_ptr< ShapeModel::Instance> GearDemo::FindFreeGear() {
    std::string type = "gear";
    std::vector<boost::shared_ptr< ShapeModel::Instance> > instances = _shapes.TypeInstances(type);
    for (size_t i = 0; i < instances.size(); i++) {

        if (instances[i]->properties["state"] == "free")
            return instances[i];
    }
    return NULL;
}

GearDemo::OpenHolderSlot GearDemo::FindFreeGearHolder(std::string geartype, bool bFill) {

    std::string vesseltype = "holder";
    std::vector<boost::shared_ptr< ShapeModel::Instance> > vinstances = _shapes.TypeInstances(vesseltype);
    for (size_t i = 0; i < vinstances.size(); i++) {
        std::vector<std::string> slots = _shapes.GetChildrenNames("parts." + vinstances[i]->metatype + ".contains");
        std::vector<std::string> slotnames = _shapes.GetShapeChildrenNames(vinstances[i]->metatype, "contains");
        for (size_t j = 0; j < slotnames.size(); j++) {
            // Check that type == holder and metatype == metatype
            std::string type = _shapes.GetChildValue<std::string>(slotnames[j] + ".type");
            std::string metatype = _shapes.GetChildValue<std::string>(slotnames[j] + ".metatype");
            if (type != "holder" || metatype != geartype)
                continue;
            std::string statechild = vinstances[i]->propname + ".contains." + slots[j] + ".state";
            std::string state = _shapes.GetChildValue<std::string>(statechild);
            if (state == "full")
                continue;
            if (bFill) {
                _shapes.PutChildValue<std::string>(statechild, "full");
            }
            return std::make_pair(vinstances[i], _shapes.GetChildPose(slotnames[j]));
        }
    }
    throw MotionException(1030, "");
}

void GearDemo::Setup() {

    // Find and make scene objects of all free gears
    std::string type = "gear";
    std::vector<boost::shared_ptr< ShapeModel::Instance> > instances = _shapes.TypeInstances(type);
    for (size_t i = 0; i < instances.size(); i++) {

        //        if (instances[i]->properties["state"] != "free")
        //            continue;

        std::string gearname = instances[i]->name; // Globals.StrFormat("%s%d", sku.c_str(), i + 1);
        tf::Pose gearpose = _baseoffset * _shapes.GetInstancePose(instances[i]);

        ShapeModel::Shape shape = _shapes.GetInstanceShape(instances[i]);
        SceneObject &obj = pScene->CreateMesh(gearname, "gear",
                pScene->gid++,
                gearpose,
                shape.meshfile,
                Scene::GetColor(shape.color),
                shape.scale);
        std::vector<double> graspoffset = _shapes.GetChildValues<double>("parts." + shape.name + ".gripper.offset");
        obj.gripperoffset = Conversion::CreateRPYPose(graspoffset);
        obj.instance = instances[i];
        //        pScene->CreateMarker("maker",
        //                Convert<tf::Pose, Eigen::Affine3d>(gearpose * Conversion::CreatePose(tf::Vector3(0.0, 0.0, 0.04))
        //                ),
        //                "GREEN");
    }

    // Find and make scene objects of all gear holders (kits or vessel)  
    type = "holder";
    instances = _shapes.TypeInstances(type);
    for (size_t i = 0; i < instances.size(); i++) {

        std::string holdername = instances[i]->name;

        // adjust pose into robot coordinates (baseoffset) 
        tf::Pose gearpose = _baseoffset * _shapes.GetInstancePose(instances[i]);

        ShapeModel::Shape shape = _shapes.GetInstanceShape(instances[i]);
        //tf::Vector3 pos = shapes.GetInstancePosition(instances[i]); // +tf::Vector3(0.0455, 0.0356,0.0);
        //        pos.rotate(tf::Vector3(0.0,0.0,1.0), Deg2Rad(-90.0));
        SceneObject &obj = pScene->CreateMesh(holdername,
                "gearholder",
                pScene->gid++,
                gearpose,
                shape.meshfile,
                Scene::GetColor(shape.color),
                shape.scale);
        obj.instance = instances[i];

    }
    // Debug: LOG_DEBUG << SceneObject::DumpDB();   
}

void GearDemo::Reset() {
    //    pScene->ClearScene(); // erases all markers
    //    Setup(); // recreates all markers
    //    pScene->DrawScene(); // draws rviz scene with markers
}

void GearDemo::Cycle(boost::shared_ptr<RCS::CController> nc, CrclApi &robot) {
    ros::Rate r(50);
    while (IssueRobotCommands(robot)) {
        while (nc->IsBusy()) {
            if (rvizdemo.Clicked()) {
                RCS::Thread::SuspendAll();
                while (!rvizdemo.Clicked())
                    r.sleep();
                RCS::Thread::ResumeAll();
            }
            ros::spinOnce();
            ros::spinOnce();
            r.sleep();
        }
    }
}
//////////////////////////////////////////////////////////////////////////
#include "nist_robotsnc/RvizMarker.h"

#ifdef CHECKERS

CheckersGame::CheckersGame(ros::NodeHandle & nh) : _nh(nh) {
    rvizgame = boost::shared_ptr<RvizCheckers> (new RvizCheckers(nh));

}

void CheckersGame::PhysicalMove(CrclApi &robot,
        int player,
        int i, int j,
        Checkers::Move m,
        bool doublejump) {

    std::string typemove("move");

    try {
        // If blank space there can be no actual checkername2 object
        std::string checkername1 = Globals.StrFormat("Checker[%d:%d]", i, j);
        std::string checkername2 = Globals.StrFormat("Checker[%d:%d]", m.row, m.col);
        tf::Pose frompose = rvizgame->GetPose(i, j);
        tf::Pose topose = rvizgame->GetPose(m.row, m.col);
        SceneObject & obj = pScene->Find(checkername1);
        if (Scene::IsNull(obj))
            throw std::runtime_error(Globals.StrFormat("PhysicalMove null scene object Checker[%d:%d]", i, j));

#ifdef DEBUG
        //        LOG_DEBUG << "Move From " << Globals.StrFormat("[%d,%d]=%f,%f", i, j, frompose(0, 3), frompose(1, 3));
        //        LOG_DEBUG << "Move To   " << Globals.StrFormat("[%d,%d]=%f,%f", m.row, m.col, topose(0, 3), topose(1, 3));
        LOG_DEBUG << "Move From " << Globals.StrFormat("[%d,%d]=%f,%f", i, j, frompose.getOrigin().x(), frompose.getOrigin().y());
        LOG_DEBUG << "Move To   " << Globals.StrFormat("[%d,%d]=%f,%f", m.row, m.col, topose.getOrigin().x(), topose.getOrigin().y());
        LOG_DEBUG << "tf    Checkerboard1 pose" << RCS::DumpPoseSimple(obj.pose);
#endif

        robot.Pick(obj.pose, checkername1);
        robot.Place(topose, checkername1);

        while (robot.cnc()->IsBusy())
            ros::Duration(0.01).sleep();

        if (m.bJump) {
            typemove = "jump";
            Checkers::Move jumped = m.Diff(Checkers::Move(i, j));
            std::string jumpedcheckername = Globals.StrFormat("Checker[%d:%d]", jumped.row, jumped.col);
            pScene->DeleteObject(jumpedcheckername);
        }
        // change checker name from old checkerboard spot to new one
        obj.name = checkername2;
        obj.pose = topose;

        if (rvizgame->IsKing(m)) {
            // Double checker height - for now 
            // Only 2* level for king
            if (obj.height == rvizgame->HEIGHT) {
                obj.height *= 2;
                obj.pose.setOrigin(obj.pose.getOrigin() + tf::Vector3(0, 0, 0.01));
                ros::spinOnce();
                ros::spinOnce();
                ros::spinOnce();
            }
        }
        ros::spinOnce();

        if (m.doublejumps.size() > 0) {
            PhysicalMove(robot, player, m.row, m.col, m.doublejumps[0], true);
        }

        // Done - move to safe non-collision position - skip if double jump
        if (!doublejump) {
            std::vector<unsigned long> jointnum(robot.cnc()->Kinematics()->NumJoints());
            std::iota(jointnum.begin(), jointnum.end(), 0); // adjusted already to 0..n-1
            robot.MoveJoints(jointnum, robot.cnc()->NamedJointMove["Safe"]);
            while (robot.cnc()->IsBusy()) {
                ros::spinOnce();
                ros::Duration(0.01).sleep();
            }
        }
    } catch (std::runtime_error &e) {
        std::cout << e.what() << "\n";
        throw;
    }

}

void CheckersGame::Setup() {
    rvizgame->RvizBoardSetup();
    rvizgame->RvizPiecesSetup();
    pScene->DrawScene(); // Debug: LOG_DEBUG << SceneObject::DumpDB();
}

void CheckersGame::Play(CrclApi * red, CrclApi * black) {
    //CrclApi * Ncs[2]={&nccmds[0], &nccmds[1]};
    //CrclApi * Ncs[2]={&fanucrobot, &fanucrobot};
    //CrclApi * Ncs[2]={&motomanrobot, &motomanrobot};

    // Play checkers - only move markers, no robot interaction
    Checkers::Move from, to;
    int player;
    for (size_t i = 0; i < 40; i++) {
        bool bQuit = rvizgame->CheckersMove(player, from, to);

        rvizgame->Game().printDisplayFancy(rvizgame->Game().Board());
        if (player == Checkers::RED) {
            LOG_DEBUG << "RED Move " << red->cnc()->Name().c_str();
            PhysicalMove(*red, player, from.row, from.col, to);
        } else {
            LOG_DEBUG << "BLACK Move " << black->cnc()->Name().c_str();
            PhysicalMove(*black, player, from.row, from.col, to);
        }
        if (bQuit)
            break;

#if 0
        // Synchronize with rviz let PublishPoint pause execution
        if (rvizdemo.Clicked()) {
            for (size_t j = 0; j < Ncs.size(); j++)
                Ncs[0]->cnc()->Suspend();
            while (1) {
                ros::spinOnce();
                ros::Duration(0.2).sleep();
                if (rvizdemo.Clicked())
                    break;
            }
            for (size_t j = 0; j < Ncs.size(); j++)
                Ncs[0]->cnc()->Resume();

        }
#endif
        ros::spinOnce();
        ros::spinOnce();
        ros::Duration(0.2).sleep();
    }
}

#endif

#ifdef EXERCISE_DEMO

void ExerciseDemo::MarkPose(int flag, tf::Pose pose) {
    pose =Robot()->cnc()->AddBaseTransform(pose); // Robot()->cnc()->basePose() * pose;
    if (flag == 1)
        RvizMarker()->SetColor(GoodColor()[0], GoodColor()[1], GoodColor()[2], 1.0);
    else if (flag == 0)
        RvizMarker()->SetColor(BadColor()[0], BadColor()[1], BadColor()[2], 1.0);
    else if (flag == -1)
        RvizMarker()->SetColor(TrapColor()[0], TrapColor()[1], TrapColor()[2], 1.0);
    //RvizMarker()->SetColor(0.597,0.0,0.597, 1.0);  // purple
    RvizMarker()->Send(pose);
}
#include <boost/bind.hpp>

void ExerciseDemo::Exercise(CrclApi *robot) {
    pScene->ClearScene();
    RvizMarker()->Clear();
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
    Globals.Sleep(1000);
    Robot() = robot;
    Robot()->cnc()->Kinematics()->ENormalize(-M_PI, M_PI);
    std::vector<size_t> scorecard = Robot()->cnc()->Kinematics()->VerifyKinematics(boost::bind(&ExerciseDemo::MarkPose, this, _1, _2));
    int total = std::accumulate(scorecard.begin(), scorecard.end(), 0);
    //int fail = std::accumulate(scorecard.begin()+1, scorecard.end(), 0);
    LOG_DEBUG << "Tests:" << total << ": good=" << scorecard[0]
            << ": bad=" << scorecard[1]
            << ": singular=" << scorecard[2] << "\n";
}
#endif
////////////////////////////////////////////////////
#ifdef SCRIPTDEMO

static double Fit(double min, double max, double v, double newmin, double newmax) {
    double d = (v - min) / (max - min)* (newmax - newmin) + newmin;
    return d;
}

static double Projection(double val, double min, double max, double scale, double neworigin) {
    // normalize to (0,1) but this is in meters, so need to scale
    double d = (val - min) / (max - min) * scale + neworigin;
    return d;
}

static tf::Vector3 Projection(vec3 val, vec3 min, vec3 max, double scale, vec3 neworigin) {
    // normalize to (0,1) but this is in meters, so need to scale
    vec3 v = (val - min) / (max - min) * scale + neworigin;
    return tf::Vector3(v.x(), v.y(), 0.0);
}


void ScriptingDemo::MarkPose(int flag, tf::Pose pose) {
            RvizMarker()->Send(pose);
            ros::spinOnce();
            ros::spinOnce();
            //ros::Duration(0.025).sleep();
}
void ScriptingDemo::Init(CrclApi *robot,
        std::string penholderfile,
        double penholderscale,
        std::string penfile,
    double pencilscale) {
    Robot() = robot;
    RvizMarker()->SetColor(0xCB, 0xCB, 0xAF);
    // This is constant so we don't care about it.
    RvizMarker()->publishMesh(tf::Identity(),
            penholderfile,
            penholderscale);
    // We are moving this around during drawing, so keep
    SceneObject & pen = pScene->CreateMesh(
            "pen",
            "mesh",
            0,
            tf::Pose(tf::QIdentity(), tf::Vector3(-0.0144, 0.0134, 0.0)),
            penfile,
            rgba(0xCB, 0xCB, 0xAF).GetColorRGBA(),
            pencilscale);
    pScene->DrawScene(); // Debug: LOG_DEBUG << SceneObject::DumpDB();
    
    Robot()->cnc()->QBend() = tf::Quaternion(Deg2Rad(0.), Deg2Rad(180.), Deg2Rad(0.));
    
     
}
           
void ScriptingDemo::Draw(
        std::string text,
        unsigned long fontsize,
        double scale,
        tf::Vector3 origin,
        tf::Pose transform) {
    Font2WorldScale() = scale;
    //pScene->ClearScene();
    //RvizMarker()->Clear();
    RvizMarker()->SetColor(FontColor()[0], FontColor()[1], FontColor()[2], 1.0);
    ros::spinOnce();
    ros::spinOnce();
    double z = origin.z();
     
    // PIck up pen
    SceneObject & obj = pScene->Find("pen");
    tf::Pose penoffset(tf::QIdentity(), tf::Vector3(0.0,0.0,0.12));
    //Robot()->cnc()->SetToolOffset(penoffset);
    Robot()->Pick(obj.pose*penoffset, "pen");
    Robot()->MoveTo(Convert<tf::Vector3, tf::Pose>(origin), "pen");
    std::vector<letter> s = script.makescript("", text, fontsize);

    float minx = script.extentminx;
    float miny = script.extentminy;
    float maxx = script.extentmaxx;
    float maxy = script.extentmaxy;

    //boost::tie(minx, miny, maxx, maxy) = s[0]. GetRange();
    double max = (maxx > maxy) ? maxx : maxy;
    for (size_t n = 0; n < s.size(); n++) {
        s[n].path = s[n].letter_gap_pts(20.0);
        // s[n].path = s[n].letter_pts(.05); // this works but in lines too much spread
        cspline::save2D(Globals.StrFormat("%s/letter%d.txt", Globals.ExeDirectory.c_str(), n), s[n].path);
        Robot()->cnc()->posecallback=boost::bind(&ScriptingDemo::MarkPose, this, _1, _2);

        for (size_t i = 0; i < s[n].path.size(); i++) {

            std::cout << " Pt:         " << s[n].path[i].x() << ":" << s[n].path[i].y() << "\n";

            // Normalize font from integer pixels into meters
            double x = Projection(s[n].path[i].x(), minx, maxx, scale,origin.x()); //  0.25);
            double y = Projection(s[n].path[i].y(), miny, maxy, scale, origin.y()); // -0.75);
            tf::Vector3 pt(x, y, z);
            tf::Pose pose(tf::QIdentity(), pt);
            std::cout << " Next       : " << DumpPoseSimple(pose) << "\n";
            // Transform point
            pose = transform*pose;
            std::cout << " Transformed: " << DumpPoseSimple(pose) << "\n";
            Robot()->MoveTo(pose, "pen");
            ros::Duration(0.025).sleep();
        }
    }

    std::stringstream str;
    // Fill in all letters horizontally - for each letter
    for (size_t n = 0; n < s.size(); n++) {
        s[n].horizontal_fill();
        std::string moves = s[n].dump_moves();
        Globals.WriteFile(Globals.StrFormat("%s/fill%d.txt", Globals.ExeDirectory.c_str(), n), moves);

        std::map<double, std::vector < vec3>>::iterator it = s[n].moves.begin();
        for (; it != s[n].moves.end(); it++) {
            std::cout << Globals.StrFormat("At %4.2f\n", (*it).first);
            std::vector<vec3> pts = (*it).second;
            if (pts.size() % 2 != 0) {
                std::cout << "Unequal moves\n";
                continue;
            }
            for (size_t i = 0; i < pts.size(); i = i + 2) {
                if (i >= pts.size() || (i + 1) >= pts.size())
                    continue;
                pts[i].z() = z;
                pts[i + 1].z() = z;
                std::cout << " Pt1: " << pts[i].x() << ":" << pts[i].y() << "\n";
                std::cout << " Pt2: " << pts[i + 1].x() << ":" << pts[i + 1].y() << "\n";
                //need project and transformation :()
                tf::Vector3 l1 = Projection(pts[i], vec3(minx, miny), vec3(maxx, maxy), scale, vec3(0.25, -0.75));
                tf::Vector3 l2 = Projection(pts[i + 1], vec3(minx, miny), vec3(maxx, maxy), scale, vec3(0.25, -0.75));
                //If you have data points that aren’t continuous you can simply tell gnuplot this by inserting one blank line between the data.

                str << l1.x() << " " << l1.y() << "\n";
                str << l2.x() << " " << l2.y() << "\n\n";
                RvizMarker()->publishLine(l1, l2, 0.01);
                ros::spinOnce();
                ros::spinOnce();
                ros::Duration(0.025).sleep();

            }
        }

        std::string ss = str.str();
        Globals.WriteFile(Globals.StrFormat("%s/fillingit.txt", Globals.ExeDirectory.c_str()), ss);

    }
    return;

}

#endif
///////////////////////////////////////////////////////////////////////

#ifdef PAINTDEMO
#include <boost/gil/image.hpp>
#include <boost/gil/typedefs.hpp>
#include <boost/gil/extension/io/jpeg_io.hpp>
 using namespace boost::gil;

//http://stackoverflow.com/questions/8300555/using-boost-gil-to-convert-an-image-into-raw-bytes
void PaintingDemo::Draw(CrclApi *robot,
        std::string imagefilename,
        double scale,
        tf::Pose transform) {
    rgb8_image_t img;
    jpeg_read_image(imagefilename, img);
    // Size = img.width()*img.height()

    for (int i = 0; i < img.height(); ++i) {
        rgb8_image_t::view_t::x_iterator it = img._view.row_begin(i);
        for (int j = 0; j < img.width(); ++j) {
            //boost::gil::rgb8_pixel_t & p = *img(i, j);  // error
            //rgb8_image_t::x_iterator it = img.row_begin(i);
            // use it[j] to access pixel[i][j]
            boost::gil::rgb8_pixel_t & p = it[j];
#if 0
            double r = get_color(p, red_t());
            double g =get_color(p, green_t());
            double b =get_color(p, blue_t());
            RvizMarker()->SetColor( r/255.,  g/255.,  b/255., 1.0);
#else
            int r = get_color(p, red_t());
            int g = get_color(p, green_t());
            int b = get_color(p, blue_t());
            RvizMarker()->SetColor(r, g, b);
#endif
            double x = Projection(i, 0, img.width(), 1., 0.25);
            double y = Projection(j, 0, img.height(), 1., -0.75);
            tf::Vector3 pt(x, y, 0.0);
            tf::Pose pose(tf::QIdentity(), pt);
            std::cout << " Next       : " << DumpPoseSimple(pose) << "\n";
            // Transform point
            pose = transform*pose;
            std::cout << " Transformed: " << DumpPoseSimple(pose) << "\n";
            //robot->MoveTo(pose);
            RvizMarker()->Send(pose);
            //ros::Duration(0.025).sleep();
                ros::spinOnce();
                ros::spinOnce();
        }
    }
        
}
#endif
