#include "mainwindow.h"
#include <QApplication>
#include <QFile>
#include <QMessageBox>
#include <QXmlStreamReader>
#include <QtDebug>
#include <QMessageBox>

#include <string>
#include <iostream>


#include "Globals.h"
#include "Config.h"


Q_DECLARE_METATYPE(UrdfJointParser::MAPLIST)

int main(int argc, char *argv[])
{
    std::string path(argv[0]);
    Globals.ExeDirectory = path.substr(0, path.find_last_of(Globals.mPathSeparator) + 1);
//    if(!Globals.parse(Globals.ExeDirectory+"Config.ini"))
//    {
//        QMessageBox::critical(NULL, QObject::tr("GAZEBO CLIENT"),
//                              QObject::tr("Config.ini not in executable folder"),
//                              QMessageBox::Ok, 0);

//        return -1;
//    }

    QApplication a(argc, argv);
    MainWindow w;

    try {

        ros::M_string remappings;
        remappings["__master"]="http://localhost:11311";
        remappings["__name"]=ROSPACKAGENAME;//.c_str();
        // Initialize ROS environment ROS_MASTER_URI MUST BE SET or seg fault
        //ros::init(argc, &argv[0], ROSPACKAGENAME);
        ros::init(remappings,ROSPACKAGENAME); // .c_str());

        // Check if master running if not abort with dialog
        if(! ros::master::check())
        {
            std::cerr << "ROS Master Not Running - Abrting\n";
            QCoreApplication::quit();
        }

        // Initialize ROS
       // ros::init(argc, argv, ROSPACKAGENAME);
        ros::NodeHandle nh;

        std::string urdf_xml;

        if (!nh.getParam("robot_description", urdf_xml)) {
            ROS_FATAL_NAMED("JointStateUpdater", "Could not load the xml from parameter server: %s", urdf_xml.c_str());
            return -1;
        }
        std::vector<std::string> source_list;
        nh.getParam("source_list", source_list);

        // ROS urdf handler - parses joint information from XML and publishes
        w.urdf = new UrdfJointParser(nh);


        if(w.urdf->ParseUrdfJoints(urdf_xml)<0)
        {
            QMessageBox::critical(NULL, QObject::tr("CPP JOINT STATE PUBLISHER"),
                                         QObject::tr("robot_description failed to parse"),
                                         QMessageBox::Ok, 0);
        }

        // This will provide ROS message callbacks, otherwise not heard
//        w.urdf -> subscribeSources();
//        ros::spin();

        qRegisterMetaType<UrdfJointParser::MAPLIST>("MAPLIST");

        w.uiconfigure();
        w.show();
        w.startUpdating();

        a.exec();


    }catch(std::exception &e)
    {
        std::cerr << e.what() << "\n";
    }
    return 0;
}
