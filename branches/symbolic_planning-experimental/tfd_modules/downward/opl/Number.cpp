/*
 * Number.cpp
 *
 *  Created on: May 9, 2011
 *      Author: Andreas Hertle
 */

#include "Number.h"

namespace opl
{

Number::Number(const QString& type, const QString& name)
: Element(type, name)
{

}

Number::~Number()
{
}

QString Number::toPDDL() const
{
    return name;
}


}
