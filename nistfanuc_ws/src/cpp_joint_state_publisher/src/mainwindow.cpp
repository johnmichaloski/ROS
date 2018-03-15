#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFont>


#include "Globals.h"
#include "Config.h"
extern std::string ExeDirectory;
int queryrate=10000;
float slider_max=10000;

static std::string device="GFAgie01"; //"SMS";
//QString mtc_url("http://agent.mtconnect.org/current");
//QString mtc_url("https://smstestbed.nist.gov:443/vds/current");

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    bSetup=true;


    // Use timer to publish joint state message ?
//    timer = new QTimer(this);
//    connect(timer, SIGNAL(timeout()), this, SLOT(publishJoints()));

}
void MainWindow::uiconfigure()
{

    // Connect source updates to GUI - see http://doc.qt.io/qt-5/qthread.html
    connect(urdf, &UrdfJointParser::resultUpdate, this, &MainWindow::publishJoints);

    std::map<std::string, std::map<std::string, double> >::iterator it;
    it= urdf->free_joints.begin();
    njts = urdf->free_joints.size();
    jointnames.reserve(urdf->free_joints.size());
    for(auto const& imap: urdf->free_joints)
        jointnames.push_back(imap.first);

    grid = new QGridLayout;
    QFont font("Helvetica", 9, QFont::Bold);

    // create lable and slider widgets and store into vector
    for(size_t i=0; i< njts; i++)
    {

        // add joint value text
        labels.push_back(new QLabel());
        labels.back()->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
        labels.back()->setMinimumWidth(50);
        labels.back()->setFont(font);
        grid->addWidget(labels.back(), i, 2);

        // add slider
        sliders.push_back(new QSlider(Qt::Horizontal));
        grid->addWidget(sliders.back(), i, 1);

        // add joint name text
        jointlabels.push_back(new QLabel());
        jointlabels.back()->setFont(font);
        jointlabels.back()->setAlignment(Qt::AlignRight);
        grid->addWidget(jointlabels.back(), i, 0);

    }

    // connect slider widget to slot
    for(size_t i=0; i< sliders.size(); i++)
    {
        connect(sliders[i], SIGNAL(valueChanged(int)), this, SLOT(slider1(int)));
        connect(sliders[i], SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
        sliders[i]->setRange(0,slider_max);
        sliders[i]->setSingleStep(1);
    }

    ui->centralWidget->setLayout(grid);

}

MainWindow::~MainWindow()
{
    urdf->bRunFlag=false;

    //http://doc.qt.io/qt-5/qthread.html
    urdf->quit();
    urdf->wait();

    delete urdf;

    // delete all widgets created.
    // THIS MAY NOT BE NECESSARY - but doesn't bomb

    for(size_t i=0; i< njts; i++)
    {
        delete sliders[i];
        delete labels[i];
        delete jointlabels[i];
    }
    delete grid ;
    delete ui;

}

void MainWindow::startUpdating()
{
    std::map<std::string, std::map<std::string, double> > myjoints;
    urdf->safeCopy(myjoints);

    for(size_t i=0; i< jointnames.size(); i++)
        jointlabels[i]->setText(jointnames[i].c_str());
    bSetup=true;

    publishJoints(myjoints);
    bSetup=false;

    urdf->start();
//    if(timer->interval()!= Globals.mUpdateRate)
//        timer->setInterval(Globals.mUpdateRate);
//    timer->start(Globals.mUpdateRate);

}


void MainWindow::publishJoints(std::map<std::string, std::map<std::string, double> > free_jnts)
{
    std::map<std::string, std::map<std::string, double> > myjoints ;
    urdf->safeCopy(myjoints);
    for(size_t i=0; i< jointnames.size(); i++)
    {
        double min = myjoints[jointnames[i]]["min"];
        double max = myjoints[jointnames[i]]["max"];
        double val = myjoints[jointnames[i]]["position"];

        val=std::min(val,max);
        val=std::max(val,min);
        int n  = (val - min) /(max-min) * slider_max;

        sliders[i]->setValue(n);
        labels[i]->setText(QString::number(val));
    }
}

void MainWindow::slider1(int n)
{
    if(bSetup)
        return;
    QSlider * slider = qobject_cast<QSlider *>(sender());
    std::vector<QSlider*>::iterator it;
    std::map<std::string, std::map<std::string, double> > myjoints;
    urdf->safeCopy(myjoints);
    if((it=std::find(sliders.begin(), sliders.end(), slider))!=sliders.end())
    {
        size_t index = it - sliders.begin();
        double min = myjoints[jointnames[index]]["min"];
        double max = myjoints[jointnames[index]]["max"];
        double val = min +  (max-min) * float(n)/slider_max;
        std::string strval=Nist::stringy::strconvert<double>(val);
        labels[index]->setText(strval.c_str());
        urdf->updateJointVal(jointnames[index], "position", val);
    }
}

void MainWindow::sliderReleased()
{
    if(bSetup)
        return;
    QSlider * slider = qobject_cast<QSlider *>(sender());
    std::vector<QSlider*>::iterator it;
    int n = slider->value();
    std::map<std::string, std::map<std::string, double> > myjoints ;
    urdf->safeCopy(myjoints);
    if((it=std::find(sliders.begin(), sliders.end(), slider))!=sliders.end())
    {
        size_t index = it - sliders.begin();
        double min = myjoints[jointnames[index]]["min"];
        double max = myjoints[jointnames[index]]["max"];
        double val = min +  (max-min) * float(n)/slider_max;
        urdf->updateJointVal(jointnames[index], "position", val);
        std::string strval=Nist::stringy::strconvert<double>(val);

        // Update slider label
        labels[index]->setText(strval.c_str());
    }
}

void MainWindow::closeEvent(QCloseEvent *ev)
{

}
