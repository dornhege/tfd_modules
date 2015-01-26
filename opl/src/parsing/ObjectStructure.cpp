/*
 * ObjectStructure.cpp
 *
 *  Created on: May 10, 2011
 *      Author: Andreas Hertle
 */

#include "ObjectStructure.h"
#include "Object.h"

namespace opl
{

ObjectStructure::ObjectStructure()
{
    keyWord = "Object";
    bodyConfigurationKeywords.append("Initialization");
}

ObjectStructure::~ObjectStructure()
{
}

Element* ObjectStructure::identify(const ParseUnit* unit) const
{
    if (unit->getTokens().size() != 2)
        return NULL;
    QStringListIterator tokenIterator(unit->getTokens());
    QString type = tokenIterator.next();
    QString name = tokenIterator.next();

    if (unit->hasArguments())
        return NULL;

    if (unit->hasScopedAccess())
        return NULL;
    return new Object(type, name);
}

}
