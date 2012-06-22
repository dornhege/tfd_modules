/*
 * NamedList.h
 *
 *  Created on: May 2, 2011
 *      Author: Andreas Hertle
 */

#ifndef NAMEDLIST_H_
#define NAMEDLIST_H_

#include "Element.h"

namespace opl
{

class NamedList: public opl::Element
{
public:
    NamedList(const QString& type, const QString& name);
    virtual ~NamedList();

    QString toPDDL() const;
};

}

#endif /* NAMEDLIST_H_ */
