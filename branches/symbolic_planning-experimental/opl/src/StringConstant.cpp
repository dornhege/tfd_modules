/*
 * StringConstant.cpp
 *
 *  Created on: May 2, 2011
 *      Author: Andreas Hertle
 */

#include "StringConstant.h"

namespace opl
{

StringConstant::StringConstant(const QString& type, const QString& name)
: Element(type, name)
{

}

StringConstant::~StringConstant()
{
}

QString StringConstant::toPDDL() const
{
    return name;
}

}
