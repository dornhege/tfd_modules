/*
 * SymbolAccessStructure.h
 *
 *  Created on: May 4, 2011
 *      Author: Andreas Hertle
 */

#ifndef SYMBOLACCESSSTRUCTURE_H_
#define SYMBOLACCESSSTRUCTURE_H_

#include "ElementStructure.h"

namespace opl
{

class SymbolAccessStructure: public opl::ElementStructure
{
public:
    SymbolAccessStructure();
    virtual ~SymbolAccessStructure();

    Element* identify(const ParseUnit* unit) const;
};

}

#endif /* SYMBOLACCESSSTRUCTURE_H_ */
