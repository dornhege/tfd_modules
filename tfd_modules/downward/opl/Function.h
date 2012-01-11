/*
 * Function.h
 *
 *  Created on: May 5, 2011
 *      Author: Andreas Hertle
 */

#ifndef FUNCTION_H_
#define FUNCTION_H_

#include "Element.h"

namespace opl
{

class Function: public opl::Element
{
public:
    Function(const QString& type, const QString& name);
    virtual ~Function();

    QString toPDDL() const;
};

}

#endif /* FUNCTION_H_ */
