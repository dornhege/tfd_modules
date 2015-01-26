/*
 * Parser.h
 *
 *  Created on: Apr 20, 2011
 *      Author: Andreas Hertle
 */

#ifndef PARSER2_H_
#define PARSER2_H_

#include <QRegExp>
#include <QStringList>

#include "parsing/ParseUnit.h"

namespace opl
{

class Parser
{
public:
    Parser();
    virtual ~Parser();

    ParseUnit* parse(QStringListIterator& tokenIterator);

    void dumpContenxt(QStringListIterator tokenIterator, int tokenCount = 20) const;
};

}

#endif /* PARSER2_H_ */
