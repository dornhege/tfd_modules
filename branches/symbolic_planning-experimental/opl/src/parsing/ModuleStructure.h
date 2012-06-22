/*
 * ModuleStructure.h
 *
 *  Created on: Apr 28, 2011
 *      Author: Andreas Hertle
 */

#ifndef MODULESTRUCTURE_H_
#define MODULESTRUCTURE_H_

#include "ElementStructure.h"

namespace opl
{

class ModuleStructure: public opl::ElementStructure
{
public:
    ModuleStructure();
    virtual ~ModuleStructure();

    Element* identify(const ParseUnit* unit) const;
};

}

#endif /* MODULESTRUCTURE_H_ */
