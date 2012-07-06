#ifndef CONTINUALPLANNINGMONITOR_WINDOW_H
#define CONTINUALPLANNINGMONITOR_WINDOW_H

#include <QMainWindow>
#include "ui_ContinualPlanningMonitorWindow.h"
#include "continual_planning_executive/ContinualPlanningStatus.h"
#include <ros/ros.h>

class ContinualPlanningMonitorWindow : public QMainWindow, protected Ui::ContinualPlanningMonitorWindow
{
    Q_OBJECT

    public:
        ContinualPlanningMonitorWindow();
        ~ContinualPlanningMonitorWindow();

    private Q_SLOTS:
        void on_actionExit_activated();

    protected:
        /// restyle the dynamically styled widgets
        void restyle();

        void statusCallback(const continual_planning_executive::ContinualPlanningStatus & status);

    private:
        continual_planning_executive::ContinualPlanningStatus st;
        ros::Subscriber _subStatus;
};

#endif

