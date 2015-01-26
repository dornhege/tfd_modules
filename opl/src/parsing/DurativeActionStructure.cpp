/*
 * DurativeActionStructure.cpp
 *
 *  Created on: May 4, 2011
 *      Author: Andreas Hertle
 */

#include "DurativeActionStructure.h"
#include "DurativeAction.h"

namespace opl
{

DurativeActionStructure::DurativeActionStructure()
{
    keyWord = "DurativeAction";
    argumentConfigurationKeywords.append("SymbolDefinition");
    bodyConfigurationKeywords.append("Duration");
    bodyConfigurationKeywords.append("Condition");
    bodyConfigurationKeywords.append("Effect");
}

DurativeActionStructure::~DurativeActionStructure()
{
}

Element* DurativeActionStructure::identify(const ParseUnit* unit) const
{
    if (unit->getTokens().size() != 2)
        return NULL;
    if (! unit->hasBody() || unit->hasScopedAccess())
        return NULL;
    QStringListIterator tokenIterator(unit->getTokens());
    QString type = tokenIterator.next();
    if (type != "DurativeAction")
        return NULL;
    QString name = tokenIterator.next();

    return new DurativeAction(type, name);
}

}
