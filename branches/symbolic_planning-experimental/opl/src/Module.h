/*
 * Module.h
 *
 *  Created on: Mar 16, 2011
 *      Author: Andreas Hertle
 */

#ifndef MODULE_H_
#define MODULE_H_

#include "Element.h"

namespace opl
{

class Module: public opl::Element
{
    const Element* libraryCall;
    const Element* effects;
public:
    Module(const QString& type, const QString& name);
    virtual ~Module();

    void customEvaluation();
    bool definesSymbol() const;
    QString toPDDL() const;

private:
    void addNumericsToPDDL(QString& pddlString, const Element* element, const QString& objectParameter) const;
};

}

#endif /* MODULE_H_ */
