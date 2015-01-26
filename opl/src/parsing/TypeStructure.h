/*
 * TypeStructure.h
 *
 *  Created on: Apr 27, 2011
 *      Author: Andreas Hertle
 */

#ifndef TYPESTRUCTURE_H_
#define TYPESTRUCTURE_H_

#include "ElementStructure.h"

namespace opl
{

class TypeStructure: public opl::ElementStructure
{
public:
    TypeStructure();
    virtual ~TypeStructure();

    Element* identify(const ParseUnit* unit) const;
};

}

#endif /* TYPESTRUCTURE_H_ */
