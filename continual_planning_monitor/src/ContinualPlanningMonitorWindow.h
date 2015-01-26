#ifndef CONTINUALPLANNINGMONITOR_WINDOW_H
#define CONTINUALPLANNINGMONITOR_WINDOW_H

#include <QMainWindow>
#include <QThread>
#include <QSignalMapper>
#include <QSettings>
#include "ui_ContinualPlanningMonitorWindow.h"
#include "continual_planning_msgs/ContinualPlanningStatus.h"
#include "continual_planning_msgs/SetContinualPlanningControl.h"
#include "continual_planning_msgs/ExecuteActionDirectly.h"
#include <ros/ros.h>

class ExecuteActionThread : public QThread
{
    Q_OBJECT

    public:
        ExecuteActionThread();

        /// Execute the action described by actionTxt
        void executeAction(QString actionTxt);

    Q_SIGNALS:
        void actionExecutionFailed(bool success, QString title, QString message);

    protected:
        void run();

        QString _actionTxt;
        continual_planning_msgs::ExecuteActionDirectly _srv;
        ros::ServiceClient _serviceExecuteActionDirectly;
};

class ContinualPlanningControlThread : public QThread
{
    Q_OBJECT

    public:
        ContinualPlanningControlThread();

        /// Set the passed in control command
        void setContinualPlanningControl(int command);

    Q_SIGNALS:
        void controlCommandSet(bool success, QString title, QString message);

    protected:
        void run();

        continual_planning_msgs::SetContinualPlanningControl _srv;
        ros::ServiceClient _serviceContinualPlanningControl;
};

class ContinualPlanningMonitorWindow : public QMainWindow, protected Ui::ContinualPlanningMonitorWindow
{
    Q_OBJECT

    public:
        ContinualPlanningMonitorWindow();
        ~ContinualPlanningMonitorWindow();

    private Q_SLOTS:
        void on_actionExit_activated();
        void on_actionReset_activated();
        void on_actionRun_activated();
        void on_actionPause_activated();
        void on_actionStep_activated();
        void on_actionExecute_Action_activated();
        void on_actionAdd_Action_activated();
        void on_actionForce_Replanning_activated();
        void on_actionReestimate_State_activated();

        void executeAction(QString actionTxt);

        void currentPlanList_contextMenu(const QPoint & pos);
        void lastPlanList_contextMenu(const QPoint & pos);

        void notifyUser(bool success, QString title, QString message);

    protected:
        /// restyle the dynamically styled widgets
        void restyle();

        void statusCallback(const continual_planning_msgs::ContinualPlanningStatus & status);

        /// Extract "detect-object table1_loc1_room1" from 1.00: (detect-objects table1_loc1_room1) [1.00]
        QString getActionDescription(QString action);

        /// Generic context menu call for executing an action that might be described in item.
        /**
         * \param [in] item the item's text is used as the action description to execute
         * \param [in] globalPos the position to bring up the context menu
         */
        void executeActionDirectly_contextMenu(QListWidgetItem* item, const QPoint & globalPos);

        /// Bring up a QInputDialog asking to edit the given actionTxt.
        QString queryActionText(QString actionTxt);

    private:
        continual_planning_msgs::ContinualPlanningStatus st;
        ros::Subscriber _subStatus;

        ContinualPlanningControlThread _continualPlanningControlThread;
        ExecuteActionThread _executeActionThread;

        unsigned int _nCustomActions;
        QSettings* _settings;
        QSignalMapper* _signalMapper;
};

#endif

