/*
 * Initialization.h
 *
 *  Created on: Feb 4, 2011
 *      Author: Andreas Hertle
 */

#ifndef INITIALIZATION_H_
#define INITIALIZATION_H_

#include "Element.h"

namespace opl
{

class Initialization: public opl::Element
{
public:
    Initialization(const QString& type, const QString& name);
    virtual ~Initialization();

    QString toPDDL() const;
};

}

#endif /* INITIALIZATION_H_ */
