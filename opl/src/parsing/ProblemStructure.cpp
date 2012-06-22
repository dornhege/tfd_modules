/*
 * ProblemStructure.cpp
 *
 *  Created on: May 10, 2011
 *      Author: Andreas Hertle
 */

#include "ProblemStructure.h"
#include "Problem.h"

namespace opl
{

ProblemStructure::ProblemStructure()
{
    keyWord = "Problem";
    argumentConfigurationKeywords.append("SymbolDefinition");
    bodyConfigurationKeywords.append("Object");
    bodyConfigurationKeywords.append("Initialization");
    bodyConfigurationKeywords.append("Goal");
}

ProblemStructure::~ProblemStructure()
{
}

Element* ProblemStructure::identify(const ParseUnit* unit) const
{
    if (unit->getTokens().size() != 2)
        return NULL;
    QStringListIterator tokenIterator(unit->getTokens());
    QString type = tokenIterator.next();
    if (type != "Problem")
        return NULL;
    QString name = tokenIterator.next();

    if (! unit->hasArguments())
        return NULL;

    if (! unit->hasBody())
        return NULL;
    return new Problem(type, name);
}

}
