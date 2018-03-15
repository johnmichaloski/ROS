#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QSlider>
#include <QLabel>
#include <QTimer>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QCloseEvent>

#include <string>
#include <boost/format.hpp>
#include <boost/property_tree/ini_parser.hpp>
namespace pt = boost::property_tree;
#include <boost/algorithm/string.hpp>
#include <boost/shared_ptr.hpp>


#include "urdfparser.h"


//class MainWindow;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void uiconfigure();
    void startUpdating();

    UrdfJointParser * urdf;
    std::vector<std::string> jointnames;
    std::map<std::string, std::string> joints;
    size_t njts;

    bool bSetup;


public slots:
    void slider1(int);
    void sliderReleased();
    void publishJoints(std::map<std::string, std::map<std::string, double> > free_joints);


private:
    void closeEvent(QCloseEvent *bar);

    QGridLayout *grid;
    std::vector<  QSlider  *> sliders;
    std::vector<  QLabel  *> labels;
    std::vector<  QLabel  *> jointlabels;

    Ui::MainWindow *ui;
    QTimer * timer;
};


#endif // MAINWINDOW_H
