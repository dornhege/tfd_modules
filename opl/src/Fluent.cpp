/*
 * Fluent.cpp
 *
 *  Created on: Apr 27, 2011
 *      Author: Andreas Hertle
 */

#include "Fluent.h"

namespace opl
{

Fluent::Fluent(const QString& type, const QString& name)
: Element(type, name)
{
}

Fluent::~Fluent()
{
}

bool Fluent::definesSymbol() const
{
    return true;
}

void Fluent::customEvaluation()
{
    if (type != "float" && type != "boolean")
    {
        // Object fluent, look up if Type exists
        const Element* correspondingType = findType(type);
        if (correspondingType == NULL)
        {
            std::cout << "Fluent::evaluate ERROR: Object fluent " << qPrintable(type) << " " << qPrintable(name) << " defined without corresponding Type" << std::endl << std::flush;
            std::cout << "fluent: " << *this << std::endl;
            assert(correspondingType != NULL);
        }
    }
}

QString Fluent::toPDDL() const
{
    QString fluent("   (");
    if (parent->getType() == "Type")
    {
        fluent.append(parent->getName()).append("_").append(name);
        fluent.append(" ?this - ").append(parent->getName());
    }
    else
    {
        fluent.append(name);
    }
    QListIterator<Element*> argumentIterator(arguments);
    while (argumentIterator.hasNext())
    {
        fluent.append(" ").append(argumentIterator.next()->toPDDL());
    }
    QString pddlType = type;
    if (type == "float")
    {
        fluent.append(") - ").append("number");
    }
    else if (type == "boolean")
    {
        fluent.append(")");
    }
    else
    {
        fluent.append(") - ").append(type);
    }
    fluent.append("\n");
    return fluent;
}

}
