/*
 * FluentMapping.cpp
 *
 *  Created on: Jul 13, 2011
 *      Author: Andreas Hertle
 */

#include "tfd_modules/opl/FluentMapping.h"
#include "tfd_modules/opl/stringutil.h"

namespace opl
{

namespace interface
{

FluentMapping::FluentMapping(int index)
{
    this->index = index;
    this->value = -1.0;
    this->constant = false;
}

FluentMapping::FluentMapping(int index, double value, bool constant)
{
    this->index = index;
    this->value = value;
    this->constant = constant;
}

FluentMapping::~FluentMapping()
{
}

std::string FluentMapping::dump() const
{
    std::string text;
    if (constant)
    {
        text = "constant: " + StringUtil::createFromNumber(index) + "|" + StringUtil::createFromNumber(value);
    }
    else
    {
        text = "mapping to var: " + StringUtil::createFromNumber(index);
    }
    return text;
}

}

}
