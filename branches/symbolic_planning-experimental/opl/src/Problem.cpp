/*
 * Problem.cpp
 *
 *  Created on: Feb 4, 2011
 *      Author: Andreas Hertle
 */

#include "Problem.h"

namespace opl
{

Problem::Problem(const QString& type, const QString& name)
: Element(type, name)
{

}

Problem::~Problem()
{
}

QString Problem::toPDDL() const
{
    QString problem = "(define (problem ";
    problem.append(name);
    problem.append(")\n  (:domain ").append(arguments.first()->getName()).append(")\n");

    QString objects = "  (:objects\n";
    QString init = "  (:init\n";
    QString goal = "  (:goal ";

    QListIterator<Element*> objectsIterator(body);
    while (objectsIterator.hasNext())
    {
        const Element* object = objectsIterator.next();
        if (object->getName() == "Goal")
        {
            goal.append(object->getBody().first()->toPDDL());
        }
        else if (object->getType() == "Initialization")
        {
            init.append(object->toPDDL());
        }
        else
        {
            objects.append(object->toPDDL());
            QListIterator<Element*> childrenIterator(object->getBody());
            while (childrenIterator.hasNext())
            {
                init.append(childrenIterator.next()->toPDDL());
            }
        }
    }

    objects.append("  )\n");
    init.append("  )\n");
    goal.append(")\n");
    problem.append(objects);
    problem.append(init);
    problem.append(goal);
    problem.append(")\n");

    return problem;
}

}
