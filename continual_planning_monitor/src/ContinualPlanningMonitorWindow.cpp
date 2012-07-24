#include <QMessageBox>
#include <QInputDialog>
#include "ContinualPlanningMonitorWindow.h"
#include "continual_planning_executive/SetContinualPlanningMode.h"
#include "continual_planning_executive/TemporalAction.h"
#include "continual_planning_executive/ExecuteActionDirectly.h"

extern bool g_Quit;

ContinualPlanningMonitorWindow::ContinualPlanningMonitorWindow()
{
    setupUi(this);

    stateEstimationGrp->setProperty("status", "inactive");
    monitoringGrp->setProperty("status", "inactive");
    planningGrp->setProperty("status", "inactive");
    executionGrp->setProperty("status", "inactive");
    resultGrp->setProperty("status", "inactive");
    restyle();

    ros::NodeHandle nh;
    _subStatus = nh.subscribe("continual_planning_status", 10,
            &ContinualPlanningMonitorWindow::statusCallback, this);
    _serviceContinualPlanningMode =
        nh.serviceClient<continual_planning_executive::SetContinualPlanningMode>("set_continual_planning_mode");
    _serviceExecuteActionDirectly =
        nh.serviceClient<continual_planning_executive::ExecuteActionDirectly>("execute_action_directly");

    currentPlanList->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(currentPlanList, SIGNAL(customContextMenuRequested(const QPoint &)),
            this, SLOT(currentPlanList_contextMenu(const QPoint &)));
    lastPlanList->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(lastPlanList, SIGNAL(customContextMenuRequested(const QPoint &)),
            this, SLOT(lastPlanList_contextMenu(const QPoint &)));
}

ContinualPlanningMonitorWindow::~ContinualPlanningMonitorWindow()
{
}

void ContinualPlanningMonitorWindow::on_actionExit_activated()
{
    g_Quit = true;
}

void ContinualPlanningMonitorWindow::on_actionReset_activated()
{
    stateEstimationGrp->setProperty("status", "inactive");
    monitoringGrp->setProperty("status", "inactive");
    planningGrp->setProperty("status", "inactive");
    executionGrp->setProperty("status", "inactive");
    resultGrp->setProperty("status", "inactive");
    restyle();

    stateTxt->document()->setPlainText("");
    lastPlanList->clear();
    currentPlanList->clear();
    currentActionTxt->setText("");
    goalReachedTxt->setText("");
}

void ContinualPlanningMonitorWindow::on_actionRun_activated()
{
    continual_planning_executive::SetContinualPlanningMode srv;
    srv.request.mode = continual_planning_executive::SetContinualPlanningMode::Request::RUN;
    if(!_serviceContinualPlanningMode.call(srv)) {
        QMessageBox::critical(this, "SetContinualPlanningMode", "Setting ContinualPlanningMode to run failed.");
    }
}

void ContinualPlanningMonitorWindow::on_actionPause_activated()
{
    continual_planning_executive::SetContinualPlanningMode srv;
    srv.request.mode = continual_planning_executive::SetContinualPlanningMode::Request::PAUSE;
    if(!_serviceContinualPlanningMode.call(srv)) {
        QMessageBox::critical(this, "SetContinualPlanningMode", "Setting ContinualPlanningMode to pause failed.");
    }
}

void ContinualPlanningMonitorWindow::on_actionExecute_Action_activated()
{
    QString actionTxt = queryActionText("");

    executeActionDirectly(actionTxt);
}

void ContinualPlanningMonitorWindow::on_actionForce_Replanning_activated()
{
    // TODO ContinualPlanningControl
}

QString ContinualPlanningMonitorWindow::getActionDescription(QString action)
{
    QString ret = action;
    int openParen = ret.indexOf("(");
    if(openParen > 0)
        ret = ret.mid(openParen + 1);   // skip everything before (

    int closeParen = ret.indexOf(")");
    if(closeParen > 0)
        ret = ret.mid(0, closeParen);   // skip everything after (

    return ret;
}

void ContinualPlanningMonitorWindow::statusCallback(const continual_planning_executive::ContinualPlanningStatus & status)
{
    QGroupBox* grp = NULL;
    QTextDocument* doc = NULL;
    QLineEdit* lineEdit = NULL;
    QListWidget* list = NULL;
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
            list = lastPlanList;
            break;
        case continual_planning_executive::ContinualPlanningStatus::EXECUTION:
            grp = executionGrp;
            lineEdit = currentActionTxt;
            break;
        case continual_planning_executive::ContinualPlanningStatus::CURRENT_PLAN:
            // do not set status for CURRENT_PLAN
            list = currentPlanList;
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
        if(list != NULL) {
            list->clear();
            QString desc = status.description.c_str();
            QStringList parts = desc.split('\n');
            foreach(QString s, parts) {
                list->addItem(s);
            }
        }
        // execution can also update the current action in the currentPlanList
        if(status.component == continual_planning_executive::ContinualPlanningStatus::EXECUTION) {
            currentPlanList->setCurrentRow(-1);
            int itemRow = -1;
            for(int i = 0; i < currentPlanList->count(); i++) {
                QListWidgetItem* item = currentPlanList->item(i);
                if(item == NULL)
                    continue;
                if(item->text() == status.description.c_str()) {
                    itemRow = i;
                    break;
                }
            }
            currentPlanList->setCurrentRow(itemRow);
            if(itemRow < 0)
                ROS_WARN("Found no matching item for execution in currentPlan: %s.", status.description.c_str());

            lastPlanList->setCurrentRow(-1);
            itemRow = -1;
            // in currentPlan this was a full match, in lastPlan the timestamps might be different, so ignore those
            QString match = getActionDescription(status.description.c_str());
            for(int i = 0; i < lastPlanList->count(); i++) {
                QListWidgetItem* item = lastPlanList->item(i);
                if(item == NULL)
                    continue;
                if(getActionDescription(item->text()) == match) {
                    itemRow = i;
                    break;
                }
            }
            lastPlanList->setCurrentRow(itemRow);
            if(itemRow < 0)
                ROS_WARN("Found no matching item for execution in lastPlan: %s.", status.description.c_str());
        }
    }

    restyle();
}

void ContinualPlanningMonitorWindow::lastPlanList_contextMenu(const QPoint & pos)
{
    QListWidgetItem* item = lastPlanList->itemAt(pos);
    QPoint globalPos = lastPlanList->mapToGlobal(pos);

    executeActionDirectly_contextMenu(item, globalPos);
}

void ContinualPlanningMonitorWindow::currentPlanList_contextMenu(const QPoint & pos)
{
    QListWidgetItem* item = currentPlanList->itemAt(pos);
    QPoint globalPos = currentPlanList->mapToGlobal(pos);

    executeActionDirectly_contextMenu(item, globalPos);
}

void ContinualPlanningMonitorWindow::executeActionDirectly_contextMenu(QListWidgetItem* item,
        const QPoint & globalPos)
{
    QMenu myMenu;
    QAction* executeActionDirectlyAction = myMenu.addAction("Execute Action Directly");
    QAction* executeAction = myMenu.addAction("Execute Action ...");

    QAction* selectedItem = myMenu.exec(globalPos);
    if(selectedItem == NULL)
        return;

    if(selectedItem == executeAction || selectedItem == executeActionDirectlyAction) {
        QString actionTxt = "";
        if(item != NULL) {
            actionTxt = getActionDescription(item->text()).trimmed();
        }

        if(selectedItem == executeActionDirectlyAction) { // directly = don't ask/edit
            if(actionTxt.length() <= 0)
                return;
        } else {
            actionTxt = queryActionText(actionTxt);
        }

        executeActionDirectly(actionTxt);
    }
}

QString ContinualPlanningMonitorWindow::queryActionText(QString actionTxt)
{
    // the whole point of doing this manually is to be able to resize the dialog.
    QInputDialog inp(this);
    inp.setInputMode(QInputDialog::TextInput);
    inp.setWindowTitle("Execute Action Directly");
    inp.setLabelText("Action:");
    inp.setTextValue(actionTxt);
    QSize s = inp.size();
    s.setWidth(s.width() + 200);
    if(inp.exec() == QDialog::Accepted)
        actionTxt = inp.textValue().trimmed();
    else
        actionTxt = "";
    return actionTxt;
}

void ContinualPlanningMonitorWindow::executeActionDirectly(QString actionTxt)
{
    if(actionTxt.length() <= 0)
        return;

    continual_planning_executive::ExecuteActionDirectly srv;
    continual_planning_executive::TemporalAction temporalAction;
    QStringList parts = actionTxt.split(" ", QString::SkipEmptyParts);
    if(parts.size() < 1) {
        ROS_ERROR("Empty parts for action: %s", qPrintable(actionTxt));
        return;
    }
    temporalAction.name = qPrintable(parts.at(0));
    for(int i = 1; i < parts.size(); i++) {
        temporalAction.parameters.push_back(qPrintable(parts.at(i)));
    }
    temporalAction.start_time = 0;  // make it executable now
    temporalAction.duration = 1;
    srv.request.action = temporalAction;
    if(!_serviceExecuteActionDirectly.call(srv)) {
        QMessageBox::critical(this, "ExecuteAction",
                QString("Executing action %1 failed.").arg(actionTxt));
    }
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

