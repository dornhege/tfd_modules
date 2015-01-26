/*
 * ElementDetector.cpp
 *
 *  Created on: Apr 27, 2011
 *      Author: Andreas Hertle
 */

#include <assert.h>
#include <iostream>
#include "ElementDetector.h"

namespace opl
{

ElementDetector::ElementDetector()
{
}

ElementDetector::~ElementDetector()
{
}

Element* ElementDetector::detect(const ParseUnit* unit, const QStringList possibleStructures) const
{
    QStringListIterator structureIterator(possibleStructures);
    while (structureIterator.hasNext())
    {
        QString structureName = structureIterator.next();
        ElementStructure* structure = structureDefinitions.value(structureName);
        if (structure == NULL)
        {
            std::cout << "ElementDetector::detect: ERROR: requested structure " << qPrintable(structureName) << " was not defined." << std::endl << std::flush;
            return NULL;
        }
        else
        {
            Element* element = structure->identify(unit);
            if (element != NULL)
            {
                // parse arguments
                if (unit->hasArguments())
                {
                    const QStringList& keywords = structure->getArgumentConfigurationKeywords();
                    QListIterator<ParseUnit*> iterator(unit->getArguments());
                    while (iterator.hasNext())
                    {
                        element->addArgument(detect(iterator.next(), keywords));
                    }
                }
                // parse body
                if (unit->hasBody())
                {
                    const QStringList& keywords = structure->getBodyConfigurationKeywords();
                    QListIterator<ParseUnit*> iterator(unit->getBody());
                    while (iterator.hasNext())
                    {
                        element->addToBody(detect(iterator.next(), keywords));
                    }
                }
                // parse scoped access
                if (unit->hasScopedAccess())
                {
                    element->setSocpedAccess(detect(unit->getScopedAccess(), structure->getScopedAccessConfigurationKeywords()));
                }
                return element;
            }
        }
    }
    std::cout << "ElementDetector::detect: ERROR: structure could not be identified:\n" << *unit << std::endl << std::flush;
    assert(0);
}

}
