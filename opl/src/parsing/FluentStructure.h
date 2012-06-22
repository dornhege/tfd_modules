/*
 * FluentStructure.h
 *
 *  Created on: Apr 27, 2011
 *      Author: Andreas Hertle
 */

#ifndef FLUENTSTRUCTURE_H_
#define FLUENTSTRUCTURE_H_

#include "ElementStructure.h"

namespace opl
{

class FluentStructure: public opl::ElementStructure
{
public:
    FluentStructure();
    virtual ~FluentStructure();

    Element* identify(const ParseUnit* unit) const;
};

}

#endif /* FLUENTSTRUCTURE_H_ */
