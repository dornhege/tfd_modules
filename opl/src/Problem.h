/*
 * Problem.h
 *
 *  Created on: Feb 4, 2011
 *      Author: Andreas Hertle
 */

#ifndef PROBLEM_H_
#define PROBLEM_H_

#include "Element.h"

namespace opl
{

class Problem : public Element
{
public:
    Problem(const QString& type, const QString& name);
    virtual ~Problem();

    QString toPDDL() const;
};

}

#endif /* PROBLEM_H_ */
