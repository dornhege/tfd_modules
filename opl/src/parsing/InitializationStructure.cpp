/*
 * InitializationStructure.cpp
 *
 *  Created on: May 10, 2011
 *      Author: Andreas Hertle
 */

#include "KeyWordList.h"
#include "InitializationStructure.h"
#include "Initialization.h"

namespace opl
{

InitializationStructure::InitializationStructure()
{
    keyWord = "Initialization";
    argumentConfigurationKeywords.append("SymbolAccess");
    bodyConfigurationKeywords.append("Number");
    bodyConfigurationKeywords.append("SymbolAccess");
}

InitializationStructure::~InitializationStructure()
{
}

Element* InitializationStructure::identify(const ParseUnit* unit) const
{
    if (unit->getTokens().size() != 1)
        return NULL;
    if (KeyWordList::isKeyWord(unit->getTokens().first()))
        return NULL;
    QStringListIterator tokenIterator(unit->getTokens());
//    QString type = tokenIterator.next();
    QString name = tokenIterator.next();

//    if (unit->hasArguments())
//        return NULL;

//    if (! unit->hasBody())
//        return NULL;

    if (unit->hasScopedAccess())
        return NULL;
    return new Initialization(keyWord, name);
}

}
