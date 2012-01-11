/*
 * Fluent.h
 *
 *  Created on: Apr 27, 2011
 *      Author: Andreas Hertle
 */

#ifndef FLUENT_H_
#define FLUENT_H_

#include "Element.h"

namespace opl
{

class Fluent: public opl::Element
{
public:
    Fluent(const QString& type, const QString& name);
    virtual ~Fluent();

    bool definesSymbol() const;
    void customEvaluation();
    QString toPDDL() const;
};

}

#endif /* FLUENT_H_ */
