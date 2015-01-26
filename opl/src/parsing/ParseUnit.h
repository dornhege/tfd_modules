/*
 * ParseUnit.h
 *
 *  Created on: Apr 21, 2011
 *      Author: Andreas Hertle
 */

#ifndef PARSEUNIT_H_
#define PARSEUNIT_H_

#include <QList>
#include <QStringList>

namespace opl
{

class ParseUnit
{
private:
    QStringList tokens;
    QList<ParseUnit*> arguments;
    QList<ParseUnit*> body;
    ParseUnit* scopedAcceess;

public:
    friend std::ostream& operator<<(std::ostream& output, const ParseUnit& element);
    ParseUnit();
    virtual ~ParseUnit();

    void addToken(const QString& token) {tokens.append(token);}
    void addArgument(ParseUnit* subElement) {arguments.append(subElement);}
    void addToBody(ParseUnit* subElement) {body.append(subElement);}
    void setScopedAccess(ParseUnit* element) {scopedAcceess = element;}

    const QStringList& getTokens() const {return tokens;}
    bool hasTokens() const {return ! tokens.empty();}
    const QList<ParseUnit*>& getArguments() const {return arguments;}
    bool hasArguments() const {return ! arguments.empty();}
    const QList<ParseUnit*>& getBody() const {return body;}
    bool hasBody() const {return ! body.empty();}
    const ParseUnit* getScopedAccess() const {return scopedAcceess;}
    bool hasScopedAccess() const {return scopedAcceess != NULL;}
};

}

#endif /* PARSEUNIT_H_ */
