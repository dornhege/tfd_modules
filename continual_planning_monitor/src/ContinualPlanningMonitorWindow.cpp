#include <QMessageBox>
#include <QInputDialog>
#include <QAction>
#include <QList>
#include <boost/thread.hpp>
#include "ContinualPlanningMonitorWindow.h"
#include "continual_planning_msgs/TemporalAction.h"

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

    currentPlanList->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(currentPlanList, SIGNAL(customContextMenuRequested(const QPoint &)),
            this, SLOT(currentPlanList_contextMenu(const QPoint &)));
    lastPlanList->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(lastPlanList, SIGNAL(customContextMenuRequested(const QPoint &)),
            this, SLOT(lastPlanList_contextMenu(const QPoint &)));

    connect(&_executeActionThread, SIGNAL(actionExecutionFailed(bool, QString, QString)),
            this, SLOT(notifyUser(bool, QString, QString)));
    connect(&_continualPlanningControlThread, SIGNAL(controlCommandSet(bool, QString, QString)),
            this, SLOT(notifyUser(bool, QString, QString)));

    _signalMapper = new QSignalMapper(this);
    _settings = new QSettings("gki", "ContinualPlanningMonitor");
    _nCustomActions = 0;

    int size = _settings->beginReadArray("custom_actions");
    for(int i = 0; i < size; ++i) {
        _settings->setArrayIndex(i);
        QString actionTxt = _settings->value("action").toString();

        // check last is the exec action, then add sep
        if(menuExecution->actions().size() > 0) {
            if(menuExecution->actions().back() == actionExecute_Action) {
                menuExecution->addSeparator();
            }
        }

        QAction* act = menuExecution->addAction(actionTxt);

        _signalMapper->setMapping(act, actionTxt);
        connect(act, SIGNAL(triggered()),
                _signalMapper, SLOT (map()));
        connect(_signalMapper, SIGNAL(mapped(QString)),
                this, SLOT(executeAction(QString)));

        _nCustomActions++;
    }
    _settings->endArray();
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
    _continualPlanningControlThread.setContinualPlanningControl(
            continual_planning_msgs::SetContinualPlanningControl::Request::RUN);
}

void ContinualPlanningMonitorWindow::on_actionPause_activated()
{
    _continualPlanningControlThread.setContinualPlanningControl(
            continual_planning_msgs::SetContinualPlanningControl::Request::PAUSE);
}

void ContinualPlanningMonitorWindow::on_actionStep_activated()
{
    _continualPlanningControlThread.setContinualPlanningControl(
            continual_planning_msgs::SetContinualPlanningControl::Request::STEP);
}

void ContinualPlanningMonitorWindow::on_actionExecute_Action_activated()
{
    QString actionTxt = queryActionText("");

    executeAction(actionTxt);
}

void ContinualPlanningMonitorWindow::on_actionAdd_Action_activated()
{
    QString actionTxt = queryActionText("");
    if(actionTxt.length() <= 0)
        return;

    // check last is the exec action, then add sep
    if(menuExecution->actions().size() > 0) {
        if(menuExecution->actions().back() == actionExecute_Action) {
            menuExecution->addSeparator();
        }
    }

    QAction* act = menuExecution->addAction(actionTxt);

    _signalMapper->setMapping(act, actionTxt);
    connect(act, SIGNAL(triggered()),
            _signalMapper, SLOT (map()));
    connect(_signalMapper, SIGNAL(mapped(QString)),
            this, SLOT(executeAction(QString)));

    _settings->beginWriteArray("custom_actions");
    _settings->setArrayIndex(_nCustomActions);
    _settings->setValue("action", actionTxt);
    _nCustomActions++;
    _settings->endArray();
}

void ContinualPlanningMonitorWindow::executeAction(QString actionTxt)
{
    _executeActionThread.executeAction(actionTxt);
}

void ContinualPlanningMonitorWindow::on_actionForce_Replanning_activated()
{
    _continualPlanningControlThread.setContinualPlanningControl(
            continual_planning_msgs::SetContinualPlanningControl::Request::FORCE_REPLANNING);
}

void ContinualPlanningMonitorWindow::on_actionReestimate_State_activated()
{
    _continualPlanningControlThread.setContinualPlanningControl(
            continual_planning_msgs::SetContinualPlanningControl::Request::REESTIMATE_STATE);
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

void ContinualPlanningMonitorWindow::statusCallback(
        const continual_planning_msgs::ContinualPlanningStatus & status)
{
    QGroupBox* grp = NULL;
    QTextDocument* doc = NULL;
    QLineEdit* lineEdit = NULL;
    QListWidget* list = NULL;
    switch(status.component) {
        case continual_planning_msgs::ContinualPlanningStatus::STATE_ESTIMATION:
            grp = stateEstimationGrp;
            doc = stateTxt->document();
            break;
        case continual_planning_msgs::ContinualPlanningStatus::MONITORING:
            grp = monitoringGrp;
            // no txt to update
            break;
        case continual_planning_msgs::ContinualPlanningStatus::PLANNING:
            grp = planningGrp;
            list = lastPlanList;
            break;
        case continual_planning_msgs::ContinualPlanningStatus::EXECUTION:
            grp = executionGrp;
            lineEdit = currentActionTxt;
            break;
        case continual_planning_msgs::ContinualPlanningStatus::CURRENT_PLAN:
            // do not set status for CURRENT_PLAN
            list = currentPlanList;
            break;
        case continual_planning_msgs::ContinualPlanningStatus::CONTINUAL_PLANNING_FINISHED:
            grp = resultGrp;
            lineEdit = goalReachedTxt;
            break;
        default:
            ROS_ERROR("Received status with unkown component: %d", status.component);
            return;
    }
    if(grp != NULL) {
        switch(status.status) {
            case continual_planning_msgs::ContinualPlanningStatus::INACTIVE:
                grp->setProperty("status", "inactive");
                break;
            case continual_planning_msgs::ContinualPlanningStatus::ACTIVE:
                grp->setProperty("status", "active");
                break;
            case continual_planning_msgs::ContinualPlanningStatus::SUCCESS:
                grp->setProperty("status", "succeeded");
                break;
            case continual_planning_msgs::ContinualPlanningStatus::FAILURE:
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
        if(status.component == continual_planning_msgs::ContinualPlanningStatus::EXECUTION) {
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

        _executeActionThread.executeAction(actionTxt);
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
    inp.resize(s);
    if(inp.exec() == QDialog::Accepted)
        actionTxt = inp.textValue().trimmed();
    else
        actionTxt = "";
    return actionTxt;
}

void ContinualPlanningMonitorWindow::notifyUser(bool success, QString title, QString message)
{
    if(success)
        QMessageBox::information(this, title, message);
    else
        QMessageBox::critical(this, title, message);
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

ExecuteActionThread::ExecuteActionThread()
{
    ros::NodeHandle nh;

    _serviceExecuteActionDirectly =
        nh.serviceClient<continual_planning_msgs::ExecuteActionDirectly>("execute_action_directly");
}

void ExecuteActionThread::executeAction(QString actionTxt)
{
    if(isRunning() && actionTxt != _actionTxt) {
        Q_EMIT actionExecutionFailed(false, "Execute action",
                "Execute action failed as another action is still running.");
        return;
    }
    // thread isn't running, and as we are here in the main thread
    // nobody can start it.

    if(actionTxt.length() <= 0)
        return;

    continual_planning_msgs::TemporalAction temporalAction;
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
    _srv.request.action = temporalAction;
    _actionTxt = actionTxt;

    // set cur srv
    start(LowPriority);
}

void ExecuteActionThread::run()
{
    // Put this service call in a background thread to reenable
    // GUI updates from the service call
    if(!_serviceExecuteActionDirectly.call(_srv)) {
        Q_EMIT actionExecutionFailed(false, "Execute Action",
                QString("Executing action %1 failed.").arg(_actionTxt));
    } else {
        // Send an OK is too annoying?
        //Q_EMIT actionExecutionFailed(true, "Execute Action",
          //      QString("Executed action %1.").arg(_actionTxt));
    }
}

ContinualPlanningControlThread::ContinualPlanningControlThread()
{
    ros::NodeHandle nh;

    _serviceContinualPlanningControl =
        nh.serviceClient<continual_planning_msgs::SetContinualPlanningControl>
        ("set_continual_planning_control");
}

void ContinualPlanningControlThread::setContinualPlanningControl(int command)
{
    if(isRunning()) {
        Q_EMIT controlCommandSet(false, "Set ContinualPlanningControl",
                "Set ContinualPlanningControl failed as another request is still running.");
        return;
    }
    // thread isn't running, and as we are here in the main thread
    // nobody can start it.

    _srv.request.command = command;

    start(LowPriority);
}

void ContinualPlanningControlThread::run()
{
    // Put this service call in a background thread to reenable
    // GUI updates from the run
    if(!_serviceContinualPlanningControl.call(_srv)) {
        Q_EMIT controlCommandSet(false, "Set ContinualPlanningControl",
                QString("Setting ContinualPlanningControl to %1 failed.").arg(_srv.request.command));
    } else {
        // don't send for RUN, etc. we'll see that because it's running now
        if(_srv.response.command == continual_planning_msgs::SetContinualPlanningControl::Request::PAUSE)
            Q_EMIT controlCommandSet(true, "Set ContinualPlanningControl",
                    "ContinualPlanningControl paused successfully.");
    }
}

