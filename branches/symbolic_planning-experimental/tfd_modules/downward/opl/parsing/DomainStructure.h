/*
 * DomainStructure.h
 *
 *  Created on: Apr 27, 2011
 *      Author: Andreas Hertle
 */

#ifndef DOMAINSTRUCTURE_H_
#define DOMAINSTRUCTURE_H_

#include "ElementStructure.h"

namespace opl
{

class DomainStructure: public opl::ElementStructure
{
public:
    DomainStructure();
    virtual ~DomainStructure();

    Element* identify(const ParseUnit* unit) const;
};

}

#endif /* DOMAINSTRUCTURE_H_ */
