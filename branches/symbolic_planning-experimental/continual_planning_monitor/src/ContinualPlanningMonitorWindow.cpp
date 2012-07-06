#include "ContinualPlanningMonitorWindow.h"

extern bool g_Quit;

ContinualPlanningMonitorWindow::ContinualPlanningMonitorWindow()
{
    setupUi(this);

    stateEstimationGrp->setProperty("status", "inactive");
    monitoringGrp->setProperty("status", "inactive");
    planningGrp->setProperty("status", "inactive");
    executionGrp->setProperty("status", "inactive");
    resultGrp->setProperty("status", "inactive");

    ros::NodeHandle nh;
    _subStatus = nh.subscribe("continual_planning_status", 10,
            &ContinualPlanningMonitorWindow::statusCallback, this);
}

ContinualPlanningMonitorWindow::~ContinualPlanningMonitorWindow()
{
}

void ContinualPlanningMonitorWindow::on_actionExit_activated()
{
    g_Quit = true;
}

void ContinualPlanningMonitorWindow::statusCallback(const continual_planning_executive::ContinualPlanningStatus & status)
{
    QGroupBox* grp = NULL;
    QTextDocument* doc = NULL;
    QLineEdit* lineEdit = NULL;
    switch(status.component) {
        case continual_planning_executive::ContinualPlanningStatus::STATE_ESTIMATION:
            grp = stateEstimationGrp;
            doc = stateTxt->document();
            break;
        case continual_planning_executive::ContinualPlanningStatus::MONITORING:
            grp = monitoringGrp;
            // no txt to update
            break;
        case continual_planning_executive::ContinualPlanningStatus::PLANNING:
            grp = planningGrp;
            doc = lastPlanTxt->document();
            break;
        case continual_planning_executive::ContinualPlanningStatus::EXECUTION:
            grp = executionGrp;
            lineEdit = currentActionTxt;
            break;
        case continual_planning_executive::ContinualPlanningStatus::CURRENT_PLAN:
            // do not set status for CURRENT_PLAN
            doc = currentPlanTxt->document();
            break;
        case continual_planning_executive::ContinualPlanningStatus::CONTINUAL_PLANNING_FINISHED:
            grp = resultGrp;
            lineEdit = goalReachedTxt;
            break;
        default:
            ROS_ERROR("Received status with unkown component: %d", status.component);
            return;
    }
    if(grp != NULL) {
        switch(status.status) {
            case continual_planning_executive::ContinualPlanningStatus::INACTIVE:
                grp->setProperty("status", "inactive");
                break;
            case continual_planning_executive::ContinualPlanningStatus::ACTIVE:
                grp->setProperty("status", "active");
                break;
            case continual_planning_executive::ContinualPlanningStatus::SUCCESS:
                grp->setProperty("status", "succeeded");
                break;
            case continual_planning_executive::ContinualPlanningStatus::FAILURE:
                grp->setProperty("status", "failed");
                break;
            default:
                ROS_ERROR("Received status with unknown status: %d", status.status);
        }
    }

    // one can send '-' to signal that the description should not be updated
    // i.e. kept at the before state
    if(status.description != "-") {
        if(doc != NULL)
            doc->setPlainText(status.description.c_str());
        if(lineEdit != NULL)
            lineEdit->setText(status.description.c_str());
    }

    restyle();
}

void ContinualPlanningMonitorWindow::restyle()
{
    style()->unpolish(stateEstimationGrp);
    style()->polish(stateEstimationGrp);
    style()->unpolish(monitoringGrp);
    style()->polish(monitoringGrp);
    style()->unpolish(planningGrp);
    style()->polish(planningGrp);
    style()->unpolish(executionGrp);
    style()->polish(executionGrp);
    style()->unpolish(resultGrp);
    style()->polish(resultGrp);
}

