/*
 * SymbolDefinitionStructure.cpp
 *
 *  Created on: Apr 28, 2011
 *      Author: Andreas Hertle
 */

#include "SymbolDefinitionStructure.h"
#include "SymbolDefinition.h"

namespace opl
{

SymbolDefinitionStructure::SymbolDefinitionStructure()
{
    keyWord = "SymbolDefinition";
}

SymbolDefinitionStructure::~SymbolDefinitionStructure()
{
}

Element* SymbolDefinitionStructure::identify(const ParseUnit* unit) const
{
    if (unit->getTokens().size() != 2)
        return NULL;
    QStringListIterator tokenIterator(unit->getTokens());
    QString type = tokenIterator.next();
    QString name = tokenIterator.next();

    if (unit->hasArguments() || unit->hasBody() || unit->hasScopedAccess())
        return NULL;
    return new SymbolDefinition(type, name);
}

}
