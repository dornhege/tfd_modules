/*
 * NamedListStructure.cpp
 *
 *  Created on: May 2, 2011
 *      Author: Andreas Hertle
 */

#include "NamedListStructure.h"
#include "NamedList.h"

namespace opl
{

NamedListStructure::NamedListStructure()
{
}

NamedListStructure::~NamedListStructure()
{
}

Element* NamedListStructure::identify(const ParseUnit* unit) const
{
    if (unit->getTokens().size() != 1)
        return NULL;
    QStringListIterator tokenIterator(unit->getTokens());
    QString name = tokenIterator.next();
    if (name != this->name)
        return NULL;

    if (unit->hasScopedAccess() || unit->hasArguments())
        return NULL;
    return new NamedList(keyWord, this->name);
}

}
