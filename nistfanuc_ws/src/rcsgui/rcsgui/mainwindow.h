#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTableWidget>
#include "tablevector.h"
#include "rosthread.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void fillTableWidget();
    void close();
    void closeEvent(QCloseEvent *event);
    void configure();
    RosThread rosThread;
private:
    Ui::MainWindow *ui;
    QTableWidget* tableWidget;
    QTableWidget* tableWidget_2;
    QStringList m_TableHeader;
    QMenu *exitMenu;
    QMenuBar *m_menuBar;
    QTableItems tableItems;
    QTableItems jnttableItems;
};

#endif // MAINWINDOW_H
