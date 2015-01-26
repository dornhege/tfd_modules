/*
 * ModuleStructure.cpp
 *
 *  Created on: Apr 28, 2011
 *      Author: Andreas Hertle
 */

#include "ModuleStructure.h"
#include "Module.h"

namespace opl
{

ModuleStructure::ModuleStructure()
{
    keyWord = "Module";
    argumentConfigurationKeywords.append("SymbolDefinition");
    bodyConfigurationKeywords.append("StringConstant");
    bodyConfigurationKeywords.append("EffectList");
}

ModuleStructure::~ModuleStructure()
{
}


Element* ModuleStructure::identify(const ParseUnit* unit) const
{
    if (unit->getTokens().size() != 2)
        return NULL;
    QStringListIterator tokenIterator(unit->getTokens());
    QString type = tokenIterator.next();
    if (type != "CostModule" && type != "ConditionModule" && type != "EffectModule")
        return NULL;
    QString name = tokenIterator.next();

    if (unit->hasScopedAccess())
        return NULL;
    return new Module(type, name);
}

}
