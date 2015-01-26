/*
 * Element.h
 *
 *  Created on: Nov 26, 2010
 *      Author: Andreas Hertle
 */

#ifndef ELEMENT_H_
#define ELEMENT_H_

#include <iostream>
#include <assert.h>
#include <QString>
#include <QHash>
#include <QList>

namespace opl
{

class Element
{
public:
    friend std::ostream& operator<<(std::ostream& output, const Element& element);
    static QString scopeSeparator;

protected:
    QString type;
    QString name;

    Element* parent;
    QList<Element*> arguments;
    QList<Element*> body;
    Element* scopedAcceess;

public:
    Element(const QString& type, const QString& name);
    ~Element();

    const QString& getType() const {return type;}
    const QString& getName() const {return name;}

    void evaluate();
    virtual void customEvaluation();

    void addArgument(Element* element);
    void addToBody(Element* element);
    void setSocpedAccess(Element* element);

    Element* getParent() const {return parent;}
    const QList<Element*>& getArguments() const {return arguments;}
    const QList<Element*>& getBody() const {return body;}
    const Element* getScopedAccess() const {return scopedAcceess;}

//    const Element* findChild(const QString& name) const;
    virtual bool definesSymbol() const ;
    virtual bool definesSymbol(const QString& name) const;
    virtual const Element* findSymbolDefinition(const QString& name) const;
    virtual const Element* findThis() const ;
    virtual const Element* findType(const QString& type) const;

    virtual QString toPDDL() const = 0;
    virtual QString toPDDL(const QString& additionalArgument) const;

    void setParent(Element* parent) {this->parent = parent;}

private:
    void checkForNameConflicts() const;
    const Element* findNameConflict(const Element* symbol) const;
};

}

#endif /* ELEMENT_H_ */
