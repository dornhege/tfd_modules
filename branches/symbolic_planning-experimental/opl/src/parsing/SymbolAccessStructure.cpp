/*
 * SymbolAccessStructure.cpp
 *
 *  Created on: May 4, 2011
 *      Author: Andreas Hertle
 */

#include "KeyWordList.h"
#include "SymbolAccessStructure.h"
#include "SymbolAccess.h"

namespace opl
{

SymbolAccessStructure::SymbolAccessStructure()
{
    keyWord = "SymbolAccess";
    argumentConfigurationKeywords.append(keyWord);
    scopedAccessConfigurationKeywords.append(keyWord);
}

SymbolAccessStructure::~SymbolAccessStructure()
{
}

Element* SymbolAccessStructure::identify(const ParseUnit* unit) const
{
    if (unit->getTokens().size() != 1)
        return NULL;
    if (! KeyWordList::isValidName(unit->getTokens().first()))
        return NULL;
    QString name = unit->getTokens().first();

    bool isNumber = false;
    name.toDouble(&isNumber);
    if (isNumber)
        return NULL;

    if (unit->hasBody())
        return NULL;
    return new SymbolAccess(keyWord, name);
}

}
