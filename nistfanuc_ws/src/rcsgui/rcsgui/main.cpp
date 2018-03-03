#include "mainwindow.h"
#include <stdlib.h>

#include <QApplication>
#include <QSettings>
#include <QMessageBox>
#include <QCloseEvent>

#include <ros/ros.h>

#include "NIST/Globals.h"
#include "NIST/Debug.h"


CGlobals Globals;

std::vector<std::string> ParseQVariant(QVariant & value)
{
    std::vector<std::string> values;
    if (value.type() == QVariant::StringList) {
        QStringList list(value.toStringList());
        foreach( QString str, list) {
            values.push_back(str.toStdString());
        }
    } else {
        values.push_back( value.toString().toStdString());
    }
    qDebug("ParseQ %s\n", RCS::VectorDump<std::string>(values).c_str());
    return values;
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;

    ros::init(argc, &argv[0], ROSPACKAGENAME);

    // Check if master running if not abort with dialog
    if(! ros::master::check())
    {
        //QMessageBox msgBox;
        //msgBox.setText("ROS Master Not Running - Aborting.");
        //msgBox.exec();
        QMessageBox::critical(NULL, QObject::tr("ROS GUI"),
                              QObject::tr("ROS Master Not Running - Aborting"),
                              QMessageBox::Ok, 0);
        QApplication::quit(); // think this only set GUI thread  flag
        //        QCloseEvent event;
        //        w.closeEvent(&event);
        return 0;

    }
    // Check the environment variables are set.
    char* masterURI=NULL;
    masterURI = getenv("ROS_MASTER_URI");
    if (masterURI==NULL)
    {
        QMessageBox::critical(NULL, QObject::tr("ROS GUI"),
                              QObject::tr("Need to set ROS environment, Maybe try source devel/setup.bash - Aborting"),
                              QMessageBox::Ok, 0);

        QApplication::quit(); // think this only set GUI thread  flag
        return 0;
    }

    std::string path(argv[0]);
    Globals.ExeDirectory = path.substr(0, path.find_last_of('/') + 1);
    Globals._appproperties["ExeDirectory"] = Globals.ExeDirectory;
    qDebug( "Exe = %s\n ", Globals.ExeDirectory.c_str()  );
    qDebug( "argv[0] = %s\n ", argv[0] );

    QString iniPath=QString::fromUtf8((Globals.ExeDirectory  + "config.ini").c_str());
    QSettings settings( iniPath, QSettings::IniFormat );
    QVariant value  =  settings.value( "system/robots" );
    std::vector<std::string> robots = ParseQVariant(value);
    value  =  settings.value( "system/poses" );
    w.rosThread.poses = ParseQVariant(value);


    for(size_t i=0; i<robots.size(); i++)
    {
        QVariant jointnames =  settings.value( (robots[i] + "/joints").c_str());
        w.rosThread.robots.jointnames[robots[i]]= ParseQVariant(jointnames);
    }

    for(size_t i=0; i<2; i++)
    {

        qDebug("Ini Robot %s\n", RCS::VectorDump<std::string>(w.rosThread.robots.jointnames[robots[i]]).c_str());

    }

    QObject::connect(&w.rosThread, SIGNAL(finished()), &a, SLOT(quit()));
    w.configure();
    w.show();
    w.rosThread.start();
    return a.exec();
}
