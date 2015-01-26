/*
 * StringStructure.cpp
 *
 *  Created on: Apr 28, 2011
 *      Author: Andreas Hertle
 */

#include "StringStructure.h"
#include "StringConstant.h"

namespace opl
{

StringStructure::StringStructure()
{
    keyWord = "StringConstant";
}

StringStructure::~StringStructure()
{
}

Element* StringStructure::identify(const ParseUnit* unit) const
{
    if (unit->getTokens().size() != 1 || unit->hasArguments() || unit->hasBody() || unit->hasScopedAccess())
        return NULL;

    return new StringConstant(keyWord, unit->getTokens().first());
}

}
