/*
 * Initialization.cpp
 *
 *  Created on: Feb 4, 2011
 *      Author: Andreas Hertle
 */


#include "Initialization.h"

namespace opl
{

Initialization::Initialization(const QString& type, const QString& name)
: Element(type, name)
{
}

Initialization::~Initialization()
{
}

QString Initialization::toPDDL() const
{
    QString initialization = "(";
    if (parent->getType() == "Problem")
    {
        initialization.append(name);
    }
    else
    {
        const Element* type = parent->findType(parent->getType());
        const Element* fluent = type->findSymbolDefinition(name);
        initialization.append(fluent->getParent()->getName()).append("_").append(fluent->getName());
        initialization.append(" ").append(parent->getName());
    }
    QListIterator<Element*> argumentIterator(arguments);
    while (argumentIterator.hasNext())
    {
        initialization.append(" ").append(argumentIterator.next()->toPDDL());
    }
    initialization.append(")");
    QString value;
    if (! body.empty())
    {
        value = body.first()->getName();
        return QString("    (= ").append(initialization).append(" ").append(value).append(")\n");
    }
    return QString("    ").append(initialization).append("\n");
}

}
