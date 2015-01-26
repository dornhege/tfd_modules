/*
 * Object.h
 *
 *  Created on: Feb 4, 2011
 *      Author: Andreas Hertle
 */

#ifndef OBJECT_H_
#define OBJECT_H_

#include "Element.h"

namespace opl
{

class Object : public Element
{
public:
    Object(const QString& type, const QString& name);
    virtual ~Object();

    bool definesSymbol() const;
    void customEvaluation();
    QString toPDDL() const;
};

}

#endif /* OBJECT_H_ */
