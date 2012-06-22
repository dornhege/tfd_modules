/*
 * NumberStructure.cpp
 *
 *  Created on: May 9, 2011
 *      Author: Andreas Hertle
 */

#include "NumberStructure.h"
#include "Number.h"

namespace opl
{

NumberStructure::NumberStructure()
{
    keyWord = "Number";
}

NumberStructure::~NumberStructure()
{
}

Element* NumberStructure::identify(const ParseUnit* unit) const
{
    if (unit->getTokens().size() != 1 || unit->hasArguments() || unit->hasBody() || unit->hasScopedAccess())
        return NULL;
    QString name = unit->getTokens().first();
    bool isNumber = false;
    name.toDouble(&isNumber);

    return new Number(keyWord, unit->getTokens().first());
}

}
