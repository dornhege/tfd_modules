#ifndef CONTINUALPLANNINGMONITOR_WINDOW_H
#define CONTINUALPLANNINGMONITOR_WINDOW_H

#include <QMainWindow>
#include "ui_ContinualPlanningMonitorWindow.h"

class ContinualPlanningMonitorWindow : public QMainWindow, protected Ui::ContinualPlanningMonitorWindow
{
    Q_OBJECT

    public:
        ContinualPlanningMonitorWindow();
        ~ContinualPlanningMonitorWindow();

    private Q_SLOTS:
        void on_actionExit_activated();

        void on_pushButton_clicked();
};

#endif

