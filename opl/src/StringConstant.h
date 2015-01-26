/*
 * StringConstant.h
 *
 *  Created on: May 2, 2011
 *      Author: Andreas Hertle
 */

#ifndef STRINGCONSTANT_H_
#define STRINGCONSTANT_H_

#include "Element.h"

namespace opl
{

class StringConstant: public opl::Element
{
public:
    StringConstant(const QString& type, const QString& name);
    virtual ~StringConstant();

    QString toPDDL() const;
};

}

#endif /* STRINGCONSTANT_H_ */
