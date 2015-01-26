/*
 * SymbolDefinitionStructure.h
 *
 *  Created on: Apr 28, 2011
 *      Author: Andreas Hertle
 */

#ifndef SYMBOLDEFINITIONSTRUCTURE_H_
#define SYMBOLDEFINITIONSTRUCTURE_H_

#include "ElementStructure.h"

namespace opl
{

class SymbolDefinitionStructure: public opl::ElementStructure
{
public:
    SymbolDefinitionStructure();
    virtual ~SymbolDefinitionStructure();

    Element* identify(const ParseUnit* unit) const;
};

}

#endif /* SYMBOLDEFINITIONSTRUCTURE_H_ */
