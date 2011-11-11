#include "plannerInterface.h"

PlannerInterface::PlannerInterface(const std::string & domainFile) : _domainFile(domainFile), _timeout(30.0)
{
}

PlannerInterface::~PlannerInterface()
{
}

std::string PlannerInterface::PlannerResultStr(enum PlannerResult pr)
{
    switch(pr) {
        case PR_SUCCESS:
            return "PR_SUCCESS";
        case PR_SUCCESS_TIMEOUT:
            return "PR_SUCCESS_TIMEOUT";
        case PR_FAILURE_TIMEOUT:
            return "PR_FAILURE_TIMEOUT";
        case PR_FAILURE_UNREACHABLE:
            return "PR_FAILURE_UNREACHABLE";
        case PR_FAILURE_OTHER:
            return "PR_FAILURE_OTHER";
    }
    return "INVALID PLANNER RESULT";
}

