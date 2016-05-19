//#include "mainwindow.h"
//#include <QApplication>
#include <QThread>

// basic ros includes
#include "ros/ros.h"
#include "std_msgs/Int32.h"

#include "SchunkGripper.h"

//MainWindow *w;

SchunkGripper *schunkGripper;

/*
 * Thread that will run the ros loop, this thread updates
 * data in the main thread
 */
class ROSWorker : public QThread
{
    void run() { ros::spin(); }
};

void gripperCallback(const std_msgs::Int32::ConstPtr& m)
{
    std::cout << "Got gripper cmd position: " << m->data << std::endl;
    //schunkGripper->SetDesiredPosition(m->data, 0, 100);
}


int main(int argc, char *argv[])
{
    //QApplication a(argc, argv);
    //w = new MainWindow();

    /*
    schunkGripper = new SchunkGripper();

    schunkGripper->start(QThread::TimeCriticalPriority);
    std::cout << "Starting the SchunkGripper thread ..." << std::endl;
    */

    // Initialize ros and register as a listener
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/boxPackaging/gripper_cmd", 1, gripperCallback);

    // Thread the ROS worker. It spins and updates the state.
    std::cout << "starting ros thread" << std::endl;
    ROSWorker* worker = new ROSWorker();
    worker->start();

    //w->show();
    //return a.exec();

    return 0;
}
