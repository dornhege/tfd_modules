/*
 * FluentStructure.cpp
 *
 *  Created on: Apr 27, 2011
 *      Author: Andreas Hertle
 */

#include "FluentStructure.h"
#include "Fluent.h"

namespace opl
{

FluentStructure::FluentStructure()
{
    keyWord = "Fluent";
    argumentConfigurationKeywords.append("SymbolDefinition");
}

FluentStructure::~FluentStructure()
{
}


Element* FluentStructure::identify(const ParseUnit* unit) const
{
    if (unit->getTokens().size() != 2)
        return NULL;
    QStringListIterator tokenIterator(unit->getTokens());
    QString type = tokenIterator.next();
    QString name = tokenIterator.next();

    if (unit->hasBody() || unit->hasScopedAccess())
        return NULL;
    return new Fluent(type, name);
}

}
