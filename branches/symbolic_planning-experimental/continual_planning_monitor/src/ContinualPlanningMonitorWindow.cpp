#include "ContinualPlanningMonitorWindow.h"

extern bool g_Quit;

ContinualPlanningMonitorWindow::ContinualPlanningMonitorWindow()
{
    setupUi(this);
    planningGrp->setProperty("status", "active");
    monitoringGrp->setProperty("status", "failed");
}

ContinualPlanningMonitorWindow::~ContinualPlanningMonitorWindow()
{

}

void ContinualPlanningMonitorWindow::on_actionExit_activated()
{
    g_Quit = true;
}

void ContinualPlanningMonitorWindow::on_pushButton_clicked()
{
    static int i = 0;
    i++;
    i%=4;
    QString st = "inactive";
    switch(i) {
        case 1:
            st = "active";
            break;
        case 2:
            st = "succeeded";
            break;
        case 3:
            st = "failed";
            break;
    }
    qWarning("SET st: %s\n", qPrintable(st));
    planningGrp->setProperty("status", st);
    monitoringGrp->setProperty("status", st);
    style()->unpolish(monitoringGrp);
    style()->polish(monitoringGrp);
    style()->unpolish(planningGrp);
    style()->polish(planningGrp);
}

