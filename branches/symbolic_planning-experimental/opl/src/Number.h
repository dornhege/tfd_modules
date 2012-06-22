/*
 * Number.h
 *
 *  Created on: May 9, 2011
 *      Author: Andreas Hertle
 */

#ifndef NUMBER_H_
#define NUMBER_H_

#include "Element.h"

namespace opl
{

class Number: public opl::Element
{
public:
    Number(const QString& type, const QString& name);
    virtual ~Number();

    QString toPDDL() const;
};

}

#endif /* NUMBER_H_ */
