/*
 * DurativeAction.h
 *
 *  Created on: Nov 26, 2010
 *      Author: Andreas Hertle
 */

#ifndef DURATIVEACTION_H_
#define DURATIVEACTION_H_

#include "Element.h"

namespace opl
{

class DurativeAction: public opl::Element
{
private:
//    double duration;
//    DelayedParseElement* unparsedDurationModule;
//    Element* durationModule;
//    Element* condition;
//    Element* effect;

public:
    DurativeAction(const QString& type, const QString& name);
    virtual ~DurativeAction();
//    void initialize(DelayedParseElement* unparsedDurationModule, Element* condition, Element* effect);
//    void initialize(double duration, Element* condition, Element* effect);

//    void evaluate();

    bool definesSymbol() const;
    QString toPDDL() const;
};

//class DurativeActionParser: public opl::Parser
//{
//public:
//    DurativeActionParser();
//    Element* parse(QStringListIterator& tokenIterator, Element* parent);
//
//    static QString keyWord;
//};

}

#endif /* DURATIVEACTION_H_ */
