#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QApplication>
#include <QDesktopWidget>
#include <QCoreApplication>
#include <QHeaderView>
#include <QMessageBox>
#include <iostream>
#include <unistd.h>

#include "NIST/Debug.h"

void MainWindow::fillTableWidget()
{
    size_t nrow, ncolumn;
    QStringList list,listv;

    nrow = 2;
    nrow += rosThread.poses.size();

    ncolumn = 6;
    //set size of QTableWidget
    ui->tableWidget->setRowCount(nrow);
    ui->tableWidget->setColumnCount(ncolumn);

    //add header to QTableWidgetItem
    list<<"X" << "Y" << "Z"<<"Roll" << "Pitch" << "Yaw";
    ui->tableWidget->setHorizontalHeaderLabels(list);
    listv<<"Fanuc" << "Motoman" ;
    for(size_t i=0; i< rosThread.poses.size(); i++)
        listv << rosThread.poses[i].c_str(); // substr(rosThread.poses[i].find_last_of('/') + 1).c_str();

    tableItems=QTableItems(nrow,6);
    tableItems.SetRowNames(listv);
    ui->tableWidget->setVerticalHeaderLabels(listv);
    ui->tableWidget->setEditTriggers(QAbstractItemView::AllEditTriggers);
    std::string str = tableItems.Dump();
    std::cerr << str;
    for(size_t i=0; i< nrow; i++)
        for(size_t j=0; j< ncolumn; j++)
             ui->tableWidget->setItem(i, j, tableItems[i][j]);
}

void MainWindow::configure()
{
    fillTableWidget();

    // Now do the robots joints
    size_t n = rosThread.robots.maxjoints();
    jnttableItems=QTableItems(4,n);
    jnttableItems.Fill<std::string>(rosThread.robots.jointnames["fanuc"], 0);
    jnttableItems.Fill<std::string>(rosThread.robots.jointnames["motoman"], 2);
    qDebug("jnttableItems =%s\n", jnttableItems.Dump().c_str());
    rosThread._tableitems=&tableItems;
    rosThread._jnttableitems=&jnttableItems;
    ui->tableWidget_2->setRowCount(4);
    ui->tableWidget_2->setColumnCount(n);
    for(size_t i=0; i< 4; i++)
        for(size_t j=0; j< n; j++)
             ui->tableWidget_2->setItem(i, j, jnttableItems[i][j]);

// Like to resize the tables to fit their column contents, but not urgent
// QT 5.2
//    ui->tableWidget->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
//    ui->tableWidget_2->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);

}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    m_menuBar = menuBar();
    m_menuBar->setNativeMenuBar(false);

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::close()
{
    rosThread.abort=true;
    rosThread.wait(1000);
    usleep(10000);
    QApplication::quit();
}
void MainWindow::closeEvent(QCloseEvent *event)
{
    rosThread.abort=true;
    rosThread.wait(1000);
    usleep(10000);
}
