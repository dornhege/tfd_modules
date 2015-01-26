/*
 * DurativeAction.cpp
 *
 *  Created on: Nov 26, 2010
 *      Author: Andreas Hertle
 */

#include <assert.h>
#include <iostream>
#include "DurativeAction.h"

namespace opl
{

DurativeAction::DurativeAction(const QString& type, const QString& name)
: Element(type, name)
{
}

DurativeAction::~DurativeAction()
{
}

bool DurativeAction::definesSymbol() const
{
    return false;
}

QString DurativeAction::toPDDL() const
{
    QString action = "  (:durative-action ";
    QString parameters("    :parameters(");
    const Element* thisElement = findThis();
    if (thisElement != NULL)
    {
        action.append(thisElement->getName()).append("_");
        action.append(name);
        parameters.append("?this - ").append(thisElement->getName());
    }
    else
    {
        action.append(name);
    }
    QListIterator<Element*> parameterIterator(getArguments());
    while (parameterIterator.hasNext())
    {
//        parameters.append(" ");
        const Element* parameter = parameterIterator.next();
        parameters.append(parameter->toPDDL());
    }
    action.append("\n").append(parameters).append(")\n");

    action.append("    :duration(= ?duration ");
    QListIterator<Element*> it(body);
    const Element* duration = it.next()->getBody().first();
    action.append(duration->toPDDL());
//    std::cout << "DurativeAction::toPDDL INFO: duration " << qPrintable(duration->getName()) << " is " << qPrintable(duration->getType()) << std::endl << std::flush;
    action.append(")\n");

    action.append("    :condition ");
    action.append(it.next()->getBody().first()->toPDDL());

    action.append("\n    :effect ");
    action.append(it.next()->getBody().first()->toPDDL());

    action.append("\n  )\n");
    return action;
}

}
