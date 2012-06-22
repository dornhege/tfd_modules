/*
 * FunctionStructure.cpp
 *
 *  Created on: May 5, 2011
 *      Author: Andreas Hertle
 */

#include "KeyWordList.h"
#include "FunctionStructure.h"
#include "Function.h"

namespace opl
{

FunctionStructure::FunctionStructure()
{
    keyWord = "Function";
    argumentConfigurationKeywords.append(keyWord);
    argumentConfigurationKeywords.append("SymbolAccess");
}

FunctionStructure::~FunctionStructure()
{
}

Element* FunctionStructure::identify(const ParseUnit* unit) const
{
    if (unit->getTokens().size() != 1 || unit->hasBody() || unit->hasScopedAccess() || ! unit->hasArguments())
        return NULL;
    if (! KeyWordList::isFunction(unit->getTokens().first()))
        return NULL;

    return new Function(keyWord, unit->getTokens().first());
}

}
