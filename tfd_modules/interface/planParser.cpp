#include "planParser.h"
#include <stdio.h>
#include <QString>
#include <QStringList>
#include <QRegExp>
#include <sstream>
#include <ros/ros.h>

PlanParser::PlanParser()
{
}

PlanParser::~PlanParser()
{
}

bool PlanParser::parsePlan(std::istream & is, Plan & plan_)
{
    vector<DurativeAction> & plan = plan_.actions;

    bool ret = true;
    plan.clear();

    // what is failure vs. empty is
    if(!is.good())
        return false;

    char buf[4096];
    memset(buf, 0, 4096);
    while(!is.eof()) {
        is.getline(buf, 4095);
        //printf("%s\n", buf);
        if(strlen(buf) == 0)    //skip empty line
            continue;

        DurativeAction action;

        QString line = buf;
        if(line.startsWith(";"))    // comment line
            continue;

        // should read like:
        // 7.00070000: (explore robot7 t15 t16) [1.00000000]
        QStringList sl1 = line.trimmed().split(":");
        if(sl1.size() != 2) {
            ROS_WARN("unparsable plan line: \"%s\"", buf);
            ret = false;
            continue;
        }
        bool ok;
        action.startTime = -1;
        action.startTime = sl1[0].toDouble(&ok);
        if(!ok) {
            ROS_WARN("unparsable plan line: \"%s\"", buf);
            ret = false;
            continue;
        }

        QString rest = sl1[1].trimmed();
        // (explore robot7 t15 t16) [1.00000000]
        QRegExp re("\\((.*)\\) *\\[(.*)\\]");
        if(rest.indexOf(re) < 0) {
            ROS_WARN("unparsable plan line: \"%s\"", buf);
            ret = false;
            continue;
        }
        if(re.captureCount() != 2) {
            ROS_WARN("unparsable plan line: \"%s\"", buf);
            ret = false;
            continue;
        }
        QString actionDesc = re.cap(1);
        QString durationDesc = re.cap(2);
        action.duration = -1;
        action.duration = durationDesc.toDouble(&ok);
        if(!ok) {
            ROS_WARN("unparsable plan line: \"%s\"", buf);
            ret = false;
            continue;
        }

        QStringList actionS = actionDesc.split(" ", QString::SkipEmptyParts);
        if(actionS.size() < 1) {
            ROS_WARN("unparsable plan line: \"%s\"", buf);
            ret = false;
            continue;
        }

        action.name = qPrintable(actionS[0]);
        for(int i = 1; i < actionS.size(); i++) {
            action.parameters.push_back(qPrintable(actionS[i]));
        }
        plan.push_back(action);
    }

    return ret;
}

