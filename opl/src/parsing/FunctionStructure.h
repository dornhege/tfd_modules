/*
 * FunctionStructure.h
 *
 *  Created on: May 5, 2011
 *      Author: Andreas Hertle
 */

#ifndef FUNCTIONSTRUCTURE_H_
#define FUNCTIONSTRUCTURE_H_

#include "ElementStructure.h"

namespace opl
{

class FunctionStructure: public opl::ElementStructure
{
public:
    FunctionStructure();
    virtual ~FunctionStructure();

    Element* identify(const ParseUnit* unit) const;
};

}

#endif /* FUNCTIONSTRUCTURE_H_ */
