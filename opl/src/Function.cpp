/*
 * Function.cpp
 *
 *  Created on: May 5, 2011
 *      Author: Andreas Hertle
 */

#include "Function.h"

namespace opl
{

Function::Function(const QString& type, const QString& name)
: Element(type, name)
{
}

Function::~Function()
{
}

QString Function::toPDDL() const
{
    bool printItemsInNewLines = false;
    QString pddlName = name;
    if (pddlName == "equals")
    {
        pddlName = "=";
    }
    else if (pddlName == "atStart")
    {
        pddlName = "at start";
    }
    else if (pddlName == "atEnd")
    {
        pddlName = "at end";
    }
    else if (pddlName == "overall")
    {
        pddlName = "over all";
    }
    else if (pddlName == "greaterEqual")
    {
        pddlName = ">=";
    }
    else if (pddlName == "and")
    {
        printItemsInNewLines = true;
    }

    QString function("(");
    function.append(pddlName);
    QListIterator<Element*> argumentIterator(arguments);
    while (argumentIterator.hasNext())
    {
        if (printItemsInNewLines)
        {
            function.append("\n      ");
        }
        else
        {
            function.append(" ");
        }
        function.append(argumentIterator.next()->toPDDL());
    }
    function.append(")");
    return function;
}

}
