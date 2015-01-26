/*
 * NamedList.cpp
 *
 *  Created on: May 2, 2011
 *      Author: Andreas Hertle
 */

#include "NamedList.h"

namespace opl
{

NamedList::NamedList(const QString& type, const QString& name)
: Element(type, name)
{

}

NamedList::~NamedList()
{
}

QString NamedList::toPDDL() const
{
    return "<NamedListPlaceholder";
}

}
