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
        void on_actionReset_activated();
        void on_actionRun_activated();
        void on_actionPause_activated();
        void on_actionExecute_Action_activated();
        void on_actionForce_Replanning_activated();

        void currentPlanList_contextMenu(const QPoint & pos);
        void lastPlanList_contextMenu(const QPoint & pos);

    protected:
        /// restyle the dynamically styled widgets
        void restyle();

        void statusCallback(const continual_planning_executive::ContinualPlanningStatus & status);

        /// Extract "detect-object table1_loc1_room1" from 1.00: (detect-objects table1_loc1_room1) [1.00]
        QString getActionDescription(QString action);

        /// Generic context menu call for executing an action that might be described in item.
        /**
         * \param [in] item the item's text is used as the action description to execute
         * \param [in] globalPos the position to bring up the context menu
         */
        void executeActionDirectly_contextMenu(QListWidgetItem* item, const QPoint & globalPos);

        /// Execute the action described by actionTxt
        void executeActionDirectly(QString actionTxt);

        /// Bring up a QInputDialog asking to edit the given actionTxt.
        QString queryActionText(QString actionTxt);

    private:
        continual_planning_executive::ContinualPlanningStatus st;
        ros::Subscriber _subStatus;

        ros::ServiceClient _serviceContinualPlanningMode;
        ros::ServiceClient _serviceExecuteActionDirectly;
};

#endif

