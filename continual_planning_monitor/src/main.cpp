#include <QApplication>
#include "ContinualPlanningMonitorWindow.h"
#include <ros/ros.h>

bool g_Quit = false;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "continual_planning_monitor");

    QApplication app(argc, argv);

    ros::NodeHandle nh;

    ContinualPlanningMonitorWindow mw;
    mw.show();

    ros::Rate loopRate(100.0);
    while(!g_Quit && ros::ok() && mw.isVisible()) {
        ros::spinOnce();
        app.processEvents();

        loopRate.sleep();
    }

    return 0;
}
