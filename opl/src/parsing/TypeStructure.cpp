/*
 * TypeStructure.cpp
 *
 *  Created on: Apr 27, 2011
 *      Author: Andreas Hertle
 */

#include "TypeStructure.h"
#include "Type.h"

namespace opl
{

TypeStructure::TypeStructure()
{
    keyWord = "Type";
    bodyConfigurationKeywords.append("Module");
    bodyConfigurationKeywords.append("Fluent");
    bodyConfigurationKeywords.append("DurativeAction");
}

TypeStructure::~TypeStructure()
{
}

Element* TypeStructure::identify(const ParseUnit* unit) const
{
    if (! unit->hasTokens())
        return NULL;
    QStringListIterator tokenIterator(unit->getTokens());
    QString type = tokenIterator.next();
    if (type != "Type")
        return NULL;

    if (! tokenIterator.hasNext())
        return NULL;
    QString name = tokenIterator.next();

    QString superName = "Object";
    if (tokenIterator.hasNext())
    {
        if (tokenIterator.next() != ":")
            return NULL;
        if (! tokenIterator.hasNext())
            return NULL;
        superName = tokenIterator.next();
    }
    if (tokenIterator.hasNext())
        return NULL;

    if (unit->hasArguments())
        return NULL;

    return new Type(type, name, superName);
}

}
