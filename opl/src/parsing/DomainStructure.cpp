/*
 * DomainStructure.cpp
 *
 *  Created on: Apr 27, 2011
 *      Author: Andreas Hertle
 */

#include "DomainStructure.h"
#include "Domain.h"

namespace opl
{

DomainStructure::DomainStructure()
{
    keyWord = "Domain";
    bodyConfigurationKeywords.append("Type");
    bodyConfigurationKeywords.append("Module");
    bodyConfigurationKeywords.append("Fluent");
    bodyConfigurationKeywords.append("DurativeAction");
}

DomainStructure::~DomainStructure()
{
}

Element* DomainStructure::identify(const ParseUnit* unit) const
{
    if (! unit->hasTokens())
        return NULL;
    QStringListIterator tokenIterator(unit->getTokens());
    QString type = tokenIterator.next();
    if (type != "Domain")
        return NULL;

    if (! tokenIterator.hasNext())
        return NULL;
    QString name = tokenIterator.next();

    if (tokenIterator.hasNext())
        return NULL;

    if (unit->hasArguments())
        return NULL;

    if (! unit->hasBody())
        return NULL;
    return new Domain(type, name);
}

}
