#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>


// basic ros includes
#include "ros/ros.h"
#include "std_msgs/Int32.h"

#include "SchunkGripper.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:


private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
